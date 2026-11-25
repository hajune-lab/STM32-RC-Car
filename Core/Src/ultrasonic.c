/* Split from the working REBUILD_16_BT main(3).c; driving settings unchanged. */
#include "ultrasonic.h"
#include "app_hooks.h"
#include "rc_config.h"
#include "rc_math.h"
#include <math.h>

static const SensorPins pins[SENSOR_COUNT] = {
    {GPIOA,GPIO_PIN_7,GPIOA,GPIO_PIN_6},
    {GPIOA,GPIO_PIN_0,GPIOA,GPIO_PIN_1},
    {GPIOA,GPIO_PIN_4,GPIOB,GPIO_PIN_0}
};
static const float cal_a[SENSOR_COUNT]={FRONT_CAL_A,LEFT_CAL_A,RIGHT_CAL_A};
static const float cal_b[SENSOR_COUNT]={FRONT_CAL_B,LEFT_CAL_B,RIGHT_CAL_B};
static Range range[SENSOR_COUNT];
static EchoPhase echo_phase;
static SensorId active_sensor, next_side=LEFT;
static bool previous_front, fired[SENSOR_COUNT];
static uint32_t fired_at[SENSOR_COUNT], ping_at, finish_at, edge_cycle, rise_cycle;
static uint32_t cycles_us;

bool Ultrasonic_LatestGood(SensorId id,uint32_t now){
    return range[id].sequence && range[id].code==GOOD && now-range[id].finished<=RANGE_FRESH_MS;
}

bool Ultrasonic_RecentGood(SensorId id,uint32_t now,uint32_t age){
    return range[id].have_good && now-range[id].good_at<=age;
}

bool Ultrasonic_NoTarget(SensorId id,uint32_t now){
    Range *r=&range[id];
    if(r->code==NO_TARGET)return now-r->finished<=RANGE_FRESH_MS;
    return r->code==NO_FALL&&r->have_no_target&&now-r->no_target_at<=FRONT_HOLD_MS;
}

bool Ultrasonic_RecentObservation(SensorId id,uint32_t now){
    return Ultrasonic_RecentGood(id,now,ALL_STALE_MS)||Ultrasonic_NoTarget(id,now);
}

static void CompleteNoTarget(SensorId id,uint32_t now){
    Range *r=&range[id];
    r->code=NO_TARGET;r->raw=-1;r->finished=now;++r->sequence;
    r->no_target_at=now;r->have_no_target=true;
    if(r->no_target_count<1000U)++r->no_target_count;
    r->fails=0;r->good_count=0;r->close_count=0;r->short_count=0;
    if(id!=FRONT&&r->no_target_count>=2U&&r->have_good&&
       r->last_good<SIDE_WALL_LIMIT_CM&&now-r->good_at<=OPENING_HINT_MS){
        r->have_opening=true;r->opening_at=now;
    }
}

static void FinishRange(RangeCode code,float value,uint32_t now){
    Range *r=&range[active_sensor];
    r->code=code;r->raw=-1;r->started=ping_at;r->finished=now;++r->sequence;
    if(code==GOOD){
        r->no_target_count=0;
        float x=Limit(cal_a[active_sensor]*value+cal_b[active_sensor],RANGE_MIN_CM,RANGE_MAX_CM);
        bool fresh=r->have_good&&now-r->good_at<=ALL_STALE_MS;
        if(active_sensor!=FRONT&&!App_SideHistoryIsFresh(r->good_at))fresh=false;
        if(active_sensor!=FRONT){
            App_ObserveSideSample(active_sensor,x,fresh,now);
            float closing=0;
            if(fresh&&now>r->good_at){
                closing=Limit((r->last_good-x)*1000.0f/(float)(now-r->good_at),
                              0,SIDE_CLOSING_MAX_CMPS);
            }
            r->closing_cmps=fresh?0.5f*r->closing_cmps+0.5f*closing:0;
        }
        if(active_sensor!=FRONT&&fresh&&x<=r->filtered*1.3f+3.0f)r->have_opening=false;
        if(active_sensor!=FRONT&&fresh&&x>r->filtered*WALL_JUMP_RATIO+3.0f){
            if(r->rising_count&&fabsf(x-r->rise_candidate)<=0.25f*r->rise_candidate)
                ++r->rising_count;
            else {r->rise_candidate=x;r->rising_count=1U;}
            if(r->rising_count>=2U){
                r->filtered=x;r->opening_at=now;r->have_opening=true;r->rising_count=0;
            }
        }else{
            r->rising_count=0;
            r->filtered=fresh?r->filtered+0.6f*(x-r->filtered):x;
        }
        r->raw=x;r->last_good=x;r->good_at=now;r->have_good=true;
        r->fails=0;r->short_count=0;
        if(r->good_count<1000U)++r->good_count;
        if(active_sensor==FRONT){
            if(x<=FRONT_TURN_CM){if(r->close_count<10U)++r->close_count;}
            else r->close_count=0;
        }
    }else{
        r->gap_votes=0;
        if(r->fails<1000U)++r->fails;
        r->good_count=0;r->close_count=0;
        if(code==TOO_CLOSE){if(r->short_count<10U)++r->short_count;}
        else r->short_count=0;
        /* A lost side echo is only a corner hint, never a front obstacle. */
        if(active_sensor!=FRONT&&r->fails>=3U&&r->have_good&&
           r->last_good<SIDE_WALL_LIMIT_CM&&now-r->good_at<=OPENING_HINT_MS){
            r->have_opening=true;r->opening_at=now;
        }
    }
    echo_phase=RX_IDLE;finish_at=now;
}

static bool Due(SensorId id,uint32_t now){return !fired[id]||now-fired_at[id]>=SAME_SENSOR_MS;}

static SensorId SelectSensor(uint32_t now){
    bool front_priority=App_SensorFrontPriority();
    if(front_priority&&Due(FRONT,now))return FRONT;
    if(!previous_front&&Due(FRONT,now))return FRONT;
    if(Due(next_side,now))return next_side;
    SensorId other=next_side==LEFT?RIGHT:LEFT;
    if(Due(other,now))return other;
    return Due(FRONT,now)?FRONT:SENSOR_COUNT;
}

void Ultrasonic_Task(uint32_t now){
    /* HC-SR04 variants can keep Echo HIGH past the range timeout with no hit.
       Observe its return to idle; never turn a permanently HIGH pin into free space. */
    for(unsigned i=0;i<SENSOR_COUNT;++i){
        if(range[i].code==NO_FALL&&
           HAL_GPIO_ReadPin(pins[i].echo_port,pins[i].echo_pin)==GPIO_PIN_RESET)
            CompleteNoTarget((SensorId)i,now);
    }
    if(echo_phase==RX_IDLE){
        if(now-ping_at<PING_SPACING_MS||now-finish_at<QUIET_MS)return;
        SensorId id=SelectSensor(now);if(id==SENSOR_COUNT)return;
        active_sensor=id;ping_at=now;fired[id]=true;fired_at[id]=now;
        previous_front=id==FRONT;if(!previous_front)next_side=id==LEFT?RIGHT:LEFT;
        if(HAL_GPIO_ReadPin(pins[id].echo_port,pins[id].echo_pin)==GPIO_PIN_SET){
            FinishRange(STUCK_HIGH,-1,now);return;
        }
        HAL_GPIO_WritePin(pins[id].trig_port,pins[id].trig_pin,GPIO_PIN_SET);
        edge_cycle=DWT->CYCCNT;echo_phase=RX_TRIGGER;return;
    }
    uint32_t cycle=DWT->CYCCNT;
    const SensorPins *p=&pins[active_sensor];
    if(echo_phase==RX_TRIGGER){
        if((uint32_t)(cycle-edge_cycle)>=TRIG_US*cycles_us){
            HAL_GPIO_WritePin(p->trig_port,p->trig_pin,GPIO_PIN_RESET);echo_phase=RX_RISE;
        }
        return;
    }
    bool high=HAL_GPIO_ReadPin(p->echo_port,p->echo_pin)==GPIO_PIN_SET;
    if(echo_phase==RX_FALL&&!high){
        uint32_t width=(uint32_t)(cycle-rise_cycle)/cycles_us;
        float cm=(float)width/58.0f;
        RangeCode code=width<ECHO_GLITCH_US?BAD_PULSE:
                       cm<RANGE_MIN_CM?TOO_CLOSE:cm>RANGE_MAX_CM?NO_FALL:GOOD;
        FinishRange(code,cm,now);return;
    }
    if(now-ping_at>=ECHO_TIMEOUT_MS){FinishRange(echo_phase==RX_RISE?NO_RISE:NO_FALL,-1,now);return;}
    if(echo_phase==RX_RISE&&high){rise_cycle=cycle;echo_phase=RX_FALL;}
}

Range *Ultrasonic_GetRanges(void){return range;}
bool Ultrasonic_IsTriggerActive(void){return echo_phase==RX_TRIGGER;}
bool Ultrasonic_IsIdle(void){return echo_phase==RX_IDLE;}
void Ultrasonic_Init(void){
    cycles_us=HAL_RCC_GetHCLKFreq()/1000000U;
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT=0U;DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
}
