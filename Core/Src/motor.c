/* Split from the working REBUILD_16_BT main(3).c; driving settings unchanged. */
#include "motor.h"
#include "main.h"
#include "tim.h"
#include "rc_config.h"
#include "rc_math.h"
#include <stdlib.h>

typedef struct { int wanted, actual, last_sign; uint32_t off_at; } Motor;
static Motor motor[2];
static uint32_t last_motor;
static bool motor_hold;
static uint32_t motor_hold_until;

int Motor_GetActual(unsigned side){
    unsigned physical=MOTOR_SWAP_SIDES?(1U-side):side;
    int polarity=physical==0U?ENA_FORWARD_SIGN:ENB_FORWARD_SIGN;
    return motor[physical].actual*polarity;
}

static void PhysicalWrite(unsigned ch,int value){
    uint16_t mask=ch==0U?(GPIO_PIN_12|GPIO_PIN_13):(GPIO_PIN_14|GPIO_PIN_15);
    uint16_t forward=ch==0U?GPIO_PIN_13:GPIO_PIN_15;
    uint16_t backward=ch==0U?GPIO_PIN_12:GPIO_PIN_14;
    uint16_t selected=value>0?forward:(value<0?backward:0U);
    unsigned duty=(unsigned)(value<0?-value:value);
    GPIOB->BSRR=((uint32_t)(mask&~selected)<<16U)|selected;
    __HAL_TIM_SET_COMPARE(&htim4,ch==0U?TIM_CHANNEL_3:TIM_CHANNEL_4,
                         duty*(PWM_PERIOD+1U)/100U);
    motor[ch].actual=value;
}

void Motor_Stop(uint32_t now){
    for(unsigned i=0;i<2U;++i){
        if(motor[i].actual){motor[i].last_sign=Sign(motor[i].actual);motor[i].off_at=now;}
        motor[i].wanted=0;PhysicalWrite(i,0);
    }
}

void Motor_Set(int left,int right){
    int v[2]={left,right};
    for(unsigned side=0;side<2U;++side){
        unsigned ch=MOTOR_SWAP_SIDES?(1U-side):side;
        int polarity=ch==0U?ENA_FORWARD_SIGN:ENB_FORWARD_SIGN;
        int x=v[side]+(v[side]? (side==0U?LEFT_TRIM:RIGHT_TRIM)*Sign(v[side]):0);
        motor[ch].wanted=ILimit(x,-100,100)*polarity;
    }
}

void Motor_Task(uint32_t now){
    if(motor_hold){if(!Reached(now,motor_hold_until))return;motor_hold=false;}
    if(now-last_motor<MOTOR_PERIOD_MS)return;
    last_motor=now;
    for(unsigned i=0;i<2U;++i){
        Motor *m=&motor[i];int want=m->wanted;
        if(!want){
            if(m->actual){m->last_sign=Sign(m->actual);m->off_at=now;PhysicalWrite(i,0);}
            continue;
        }
        if(m->actual&&Sign(m->actual)!=Sign(want)){
            m->last_sign=Sign(m->actual);m->off_at=now;PhysicalWrite(i,0);continue;
        }
        if(!m->actual&&m->last_sign&&m->last_sign!=Sign(want)&&now-m->off_at<REVERSE_DEAD_MS)continue;
        int magnitude=ILimit(want<0?-want:want,DRIVE_MIN_PWM,100);
        int target=Sign(want)*magnitude;
        int step=(m->actual&&abs(target)<abs(m->actual))?MOTOR_DECEL_STEP:MOTOR_RAMP_STEP;
        int out=m->actual?m->actual+ILimit(target-m->actual,-step,step)
                         :Sign(want)*DRIVE_MIN_PWM;
        m->last_sign=Sign(out);PhysicalWrite(i,out);
    }
}

void Motor_Init(void){
    __HAL_TIM_DISABLE(&htim4);__HAL_TIM_SET_PRESCALER(&htim4,0U);
    __HAL_TIM_SET_AUTORELOAD(&htim4,PWM_PERIOD);
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0U);__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0U);
    __HAL_TIM_SET_COUNTER(&htim4,0U);htim4.Instance->EGR=TIM_EGR_UG;
    if(HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3)!=HAL_OK||HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4)!=HAL_OK)Error_Handler();
}

void Motor_CancelHold(void){motor_hold=false;}
void Motor_HoldUntil(uint32_t until){motor_hold=true;motor_hold_until=until;}
void Motor_TestPhysical(unsigned channel,int duty){
    motor[0].wanted=0;motor[1].wanted=0;
    if(channel<2U)motor[channel].wanted=ILimit(duty,-100,100);
}
