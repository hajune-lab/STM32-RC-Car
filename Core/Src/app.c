/* Split from the working REBUILD_16_BT main(3).c; driving settings unchanged. */
#include "app.h"
#include "app_hooks.h"
#include "board.h"
#include "motor.h"
#include "bt.h"
#include "debug_uart.h"
#include "rc_config.h"
#include "rc_math.h"
#include <math.h>
#include <stdio.h>

typedef enum { STOP, ACQUIRE, DRIVE, CORNER, BACK, TEST, MANUAL_DRIVE } DriveState;
typedef enum { MODE_MANUAL, MODE_AUTO } ControlMode;
typedef enum { TURN_POWER, TURN_FORWARD } TurnPhase;
typedef enum { NO_WALL, BOTH_WALLS, LEFT_WALL, RIGHT_WALL } WallMode;

#define range (Ultrasonic_GetRanges())

static DriveState state=STOP;
static ControlMode control_mode=MODE_MANUAL;
static int manual_left, manual_right;
static TurnPhase turn_phase;
static WallMode wall_mode=NO_WALL;
static int turn_direction, previous_direction;
/* A pending corner survives short reverses and brief apparent exits.
   Centering/near-wall steering NEVER writes this memory. */
static int pending_corner_direction;
static uint32_t corner_forward_ms, corner_track_at, corner_clear_sequence;
static uint16_t corner_clear_samples;
static uint32_t state_at, acquire_after, last_control, last_log;
static uint32_t turn_front_sequence;
static bool arc_active;
static uint32_t arc_run_ms;
static uint32_t turn_on_elapsed, turn_tick, turn_progress_at;
static uint32_t after_turn_until, side_sequence[2], steer_at;
static uint32_t side_alignment_after;
static bool side_alignment_gate;
static float turn_best, wall_target, steering, old_error;
static uint16_t turn_clear_count;
static bool close_rotation_active, stalled_pivot;
static uint32_t close_rotation_ms, close_rotation_tick;
static const char *reason="BOOT", *direction_reason="NONE";
static int test_left, test_right, test_physical=-1;
static GPIO_PinState button_idle, button_raw, button_stable;
static uint32_t button_at;
static uint32_t log_overruns;
static uint32_t fast_front_sequence;
static uint16_t fast_clear_samples;


static int CornerDirection(uint32_t now,bool allow_tie);

static bool Emergency(uint32_t now){
    return (Ultrasonic_LatestGood(FRONT,now)&&range[FRONT].raw<=FRONT_EMERGENCY_CM)||
      (range[FRONT].code==TOO_CLOSE&&range[FRONT].short_count>=2U&&
       now-range[FRONT].finished<=RANGE_FRESH_MS);
}

static void ResetCenter(void){
    wall_mode=NO_WALL;steering=0;old_error=0;steer_at=0;
    side_sequence[0]=range[LEFT].sequence;side_sequence[1]=range[RIGHT].sequence;
}

static void EnterAcquire(uint32_t now,const char *why){
    Motor_Stop(now);state=ACQUIRE;state_at=now;
    acquire_after=now+ACQUIRE_SETTLE_MS;reason=why;ResetCenter();
}

static void ClearCornerMemory(uint32_t now){
    pending_corner_direction=0;corner_forward_ms=0;corner_clear_samples=0;
    corner_track_at=now;corner_clear_sequence=range[FRONT].sequence;
}

static void RememberCorner(int direction,uint32_t now){
    if(!direction)return;
    pending_corner_direction=direction;corner_forward_ms=0;corner_clear_samples=0;
    corner_track_at=now;corner_clear_sequence=range[FRONT].sequence;
}

static void TrackCornerExit(uint32_t now){
    uint32_t dt=now-corner_track_at;corner_track_at=now;
    if(!pending_corner_direction)return;
    bool clear=(Ultrasonic_LatestGood(FRONT,now)&&range[FRONT].raw>FRONT_TURN_CM)||
               (Ultrasonic_NoTarget(FRONT,now)&&range[FRONT].no_target_count>=2U);
    bool forward=state==DRIVE&&Motor_GetActual(0)>0&&Motor_GetActual(1)>0;
    if(!forward||!clear){
        corner_forward_ms=0;corner_clear_samples=0;
        corner_clear_sequence=range[FRONT].sequence;return;
    }
    /* A delayed main-loop call must not count a long unobserved interval. */
    corner_forward_ms+=(dt>2U*CONTROL_MS)?2U*CONTROL_MS:dt;
    if(range[FRONT].sequence!=corner_clear_sequence){
        corner_clear_sequence=range[FRONT].sequence;
        if(corner_clear_samples<CORNER_CLEAR_SAMPLES)++corner_clear_samples;
    }
    if(corner_forward_ms>=CORNER_CLEAR_RUN_MS&&corner_clear_samples>=CORNER_CLEAR_SAMPLES)
        ClearCornerMemory(now);
}

static void Start(uint32_t now){
    ClearCornerMemory(now);close_rotation_active=false;stalled_pivot=false;
    control_mode=MODE_AUTO;manual_left=manual_right=0;side_alignment_gate=false;
    turn_direction=0;after_turn_until=now;fast_clear_samples=0;
    for(unsigned i=LEFT;i<SENSOR_COUNT;++i){
        range[i].have_approach=false;range[i].measured_gap=false;range[i].gap_votes=0;
    }
    EnterAcquire(now,"START_SENSE");
}

static void Stop(uint32_t now){
    ClearCornerMemory(now);close_rotation_active=false;stalled_pivot=false;
    Motor_Stop(now);Motor_CancelHold();control_mode=MODE_MANUAL;
    manual_left=manual_right=0;state=STOP;reason="USER_STOP";
}

static void ToggleMode(uint32_t now){
    if(control_mode==MODE_AUTO){Stop(now);reason="MODE_MANUAL";}
    else {Motor_Stop(now);Start(now);}
}

static void StartBack(uint32_t now,const char *why){
    /* If emergency reverse preceded corner entry, latch a direction only when
       current front/side evidence supports one. Never copy old steering signs. */
    if(!pending_corner_direction&&Emergency(now)){
        int d=(state==CORNER&&turn_direction)?turn_direction:CornerDirection(now,true);
        RememberCorner(d,now);
    }
    corner_forward_ms=0;corner_clear_samples=0;corner_track_at=now;
    close_rotation_active=false;stalled_pivot=false;arc_active=false;
    Motor_Stop(now);state=BACK;state_at=now;turn_on_elapsed=0;
    turn_tick=now;reason=why;Motor_Set(-BACK_PWM,-BACK_PWM);
}

void App_ObserveSideSample(SensorId id,float cm,bool fresh,uint32_t now){
    Range *r=&range[id];
    bool straight_approach=state==DRIVE||
        (state==CORNER&&!turn_direction&&Motor_GetActual(0)>=0&&Motor_GetActual(1)>=0);
    if(!straight_approach||control_mode!=MODE_AUTO){r->gap_votes=0;return;}
    if(!fresh||!r->have_approach||now-r->approach_at>OPENING_HINT_MS){
        r->approach_wall=cm;r->have_approach=true;r->measured_gap=false;r->gap_votes=0;
    }
    r->approach_at=now;
    if(r->measured_gap&&now-r->measured_gap_at>OPENING_HINT_MS){
        r->measured_gap=false;r->gap_votes=0;r->approach_wall=cm;
    }
    if(r->measured_gap&&cm<=r->approach_wall*1.25f+2.0f)r->measured_gap=false;
    bool near_front=Ultrasonic_RecentGood(FRONT,now,RANGE_FRESH_MS)&&
                    range[FRONT].last_good<=FRONT_SLOW_CM;
    bool grown=cm>=r->approach_wall*GAP_GROWTH_RATIO&&cm-r->approach_wall>=GAP_GROWTH_CM;
    if(near_front&&grown){
        if(r->gap_votes<GAP_CONFIRM_SAMPLES)++r->gap_votes;
        if(r->gap_votes>=GAP_CONFIRM_SAMPLES&&!r->measured_gap){
            r->measured_gap=true;r->measured_gap_at=now;
        }
    }else{
        r->gap_votes=0;
        if(!r->measured_gap)r->approach_wall+=0.12f*(cm-r->approach_wall);
    }
}

static bool MeasuredOpening(SensorId id,uint32_t now){
    Range *r=&range[id];
    return Ultrasonic_LatestGood(id,now)&&r->measured_gap&&
           now-r->measured_gap_at<=OPENING_HINT_MS&&
           r->raw-r->approach_wall>=GAP_GROWTH_CM;
}

static bool SideForSteering(SensorId id,uint32_t now){
    return Ultrasonic_LatestGood(id,now)&&(!side_alignment_gate||
           Reached(range[id].started,side_alignment_after));
}

static bool WallVisible(SensorId id,uint32_t now){
    if(side_alignment_gate&&(!Reached(range[id].good_at,side_alignment_after)||
                             !Reached(range[id].started,side_alignment_after)))return false;
    return Ultrasonic_RecentGood(id,now,RANGE_FRESH_MS)&&range[id].fails<2U&&
           !(Ultrasonic_NoTarget(id,now)&&range[id].no_target_count>=2U)&&
           range[id].filtered<=SIDE_WALL_LIMIT_CM&&range[id].rising_count==0U;
}

static float CenterSteer(uint32_t now){
    bool l=WallVisible(LEFT,now),r=WallVisible(RIGHT,now);
    WallMode selected=l?(r?BOTH_WALLS:LEFT_WALL):(r?RIGHT_WALL:NO_WALL);
    bool update=range[LEFT].sequence!=side_sequence[0]||range[RIGHT].sequence!=side_sequence[1]||selected!=wall_mode;
    if(!update)return steering;
    side_sequence[0]=range[LEFT].sequence;side_sequence[1]=range[RIGHT].sequence;
    float L=range[LEFT].filtered,R=range[RIGHT].filtered,e=0;
    if(selected==BOTH_WALLS)e=(L-R)/fmaxf(L+R,1.0f);
    else if(selected==LEFT_WALL){
        if(wall_mode!=LEFT_WALL)wall_target=L;
        e=(L-wall_target)/fmaxf(L+wall_target,1.0f);
    }else if(selected==RIGHT_WALL){
        if(wall_mode!=RIGHT_WALL)wall_target=R;
        e=(wall_target-R)/fmaxf(R+wall_target,1.0f);
    }
    if(fabsf(e)<CENTER_DEADBAND)e=0;
    float derivative=0;
    if(steer_at&&selected==wall_mode&&now>steer_at)
        derivative=Limit((e-old_error)*1000.0f/(float)(now-steer_at),-3.0f,3.0f);
    float request=CENTER_KP*e+CENTER_KD*derivative;
    steering=Limit(0.5f*steering+0.5f*request,-CENTER_MAX_PWM,CENTER_MAX_PWM);
    if(SideForSteering(LEFT,now)&&range[LEFT].raw<=SIDE_CLOSE_CM&&
       !(SideForSteering(RIGHT,now)&&range[RIGHT].raw<=SIDE_CLOSE_CM))steering=-CENTER_MAX_PWM;
    if(SideForSteering(RIGHT,now)&&range[RIGHT].raw<=SIDE_CLOSE_CM&&
       !(SideForSteering(LEFT,now)&&range[LEFT].raw<=SIDE_CLOSE_CM))steering=CENTER_MAX_PWM;
    wall_mode=selected;old_error=e;steer_at=now;return steering;
}

static void ForwardCommand(uint32_t now,bool slow){
    float steer=CenterSteer(now);
    if(!Reached(now,after_turn_until))steer=Limit(steer,-3.0f,3.0f);
    /* A separate close-wall guard; do not raise normal centering gain.
       Similar short distances on BOTH sides mean a narrow lane, not a turn. */
    bool left_valid=SideForSteering(LEFT,now),right_valid=SideForSteering(RIGHT,now);
    float L=range[LEFT].raw,R=range[RIGHT].raw;
    /* Estimate near-future clearance only from recent valid measurements.
       Limit the prediction so one short echo cannot create a large turn. */
    if(left_valid)L-=Limit(range[LEFT].closing_cmps*SIDE_LOOKAHEAD_S,0,SIDE_PREDICT_MAX_CM);
    if(right_valid)R-=Limit(range[RIGHT].closing_cmps*SIDE_LOOKAHEAD_S,0,SIDE_PREDICT_MAX_CM);
    float close_steer=0;
    if(left_valid&&L<SIDE_AVOID_START_CM&&
       (!right_valid||R-L>=SIDE_AVOID_DIFF_CM)){
        float space=right_valid?Limit((R-L-SIDE_AVOID_DIFF_CM)/6.0f,0,1):0.5f;
        close_steer=-SIDE_AVOID_MAX_PWM*space*Limit((SIDE_AVOID_START_CM-L)/
                      (SIDE_AVOID_START_CM-SIDE_AVOID_FULL_CM),0,1);
    }
    if(right_valid&&R<SIDE_AVOID_START_CM&&
       (!left_valid||L-R>=SIDE_AVOID_DIFF_CM)){
        float space=left_valid?Limit((L-R-SIDE_AVOID_DIFF_CM)/6.0f,0,1):0.5f;
        close_steer=SIDE_AVOID_MAX_PWM*space*Limit((SIDE_AVOID_START_CM-R)/
                      (SIDE_AVOID_START_CM-SIDE_AVOID_FULL_CM),0,1);
    }
    if(close_steer&&(fabsf(close_steer)>fabsf(steer)||close_steer*steer<0))steer=close_steer;
    /* Corner memory selects recovery direction only. It must not keep a
       cleared, freshly measured straight at slow speed. Alignment freshness
       still gates both sides after a turn; proximity guards remain active. */
    bool front_ready=Ultrasonic_LatestGood(FRONT,now)&&now-range[FRONT].finished<=FAST_FRONT_AGE_MS;
    bool sides_ready=left_valid&&right_valid&&
        now-range[LEFT].finished<=FAST_SIDE_AGE_MS&&
        now-range[RIGHT].finished<=FAST_SIDE_AGE_MS&&
        range[LEFT].rising_count==0U&&range[RIGHT].rising_count==0U;
    bool fast_ok=!slow&&state==DRIVE&&front_ready&&sides_ready&&
        range[FRONT].raw>=FAST_FRONT_CM&&L>=FAST_SIDE_MARGIN_CM&&R>=FAST_SIDE_MARGIN_CM&&
        fabsf(L-R)/fmaxf(L+R,1.0f)<=FAST_CENTER_ERROR&&
        range[LEFT].closing_cmps<=FAST_CLOSING_CMPS&&range[RIGHT].closing_cmps<=FAST_CLOSING_CMPS&&
        !range[LEFT].have_opening&&!range[RIGHT].have_opening;
    if(!fast_ok)fast_clear_samples=0;
    if(range[FRONT].sequence!=fast_front_sequence){
        fast_front_sequence=range[FRONT].sequence;
        if(fast_ok&&fast_clear_samples<FAST_CONFIRM_SAMPLES)++fast_clear_samples;
    }
    float base=DRIVE_SLOW_PWM;
    if(!slow&&state==DRIVE&&front_ready&&sides_ready){
        float front_space=Limit((range[FRONT].raw-FRONT_CLEAR_CM)/
                               (CRUISE_FULL_FRONT_CM-FRONT_CLEAR_CM),0,1);
        float side_space=Limit((fminf(L,R)-SIDE_CLOSE_CM)/
                              (CRUISE_SIDE_MARGIN_CM-SIDE_CLOSE_CM),0,1);
        float balance=Limit(1.0f-SPEED_BALANCE_GAIN*fabsf(L-R)/fmaxf(L+R,1.0f),0,1);
        base+=(DRIVE_CRUISE_PWM-DRIVE_SLOW_PWM)*front_space*side_space*balance;
        if(fast_ok&&fast_clear_samples>=FAST_CONFIRM_SAMPLES)base=DRIVE_FAST_PWM;
    }
    if(close_steer){
        /* Small wall corrections no longer force an immediate return to 70.
           Blend down by predicted clearance, with full slowdown by 11 cm.
           Strong avoidance also keeps full slowdown even if only one side is valid. */
        float nearest=left_valid?(right_valid?fminf(L,R):L):R;
        float distance_risk=Limit((SIDE_AVOID_START_CM-nearest)/
                                 (SIDE_AVOID_START_CM-SIDE_FULL_SLOW_CM),0,1);
        float steer_risk=Limit((fabsf(close_steer)-2.0f)/
                              (SIDE_AVOID_MAX_PWM-2.0f),0,1);
        float risk=fmaxf(distance_risk,steer_risk);
        base+=(DRIVE_SLOW_PWM-base)*risk;
    }
    /* Keep a fresh but distant wall from receiving large high-speed correction. */
    float speed_steer=Limit(steer,-CENTER_MAX_PWM,CENTER_MAX_PWM);
    if(close_steer)speed_steer=steer;
    if(left_valid&&right_valid&&fmaxf(L,R)<=SIDE_CLOSE_CM+3.0f&&
       fabsf(L-R)<=SIDE_AVOID_DIFF_CM+1.0f)speed_steer=Limit(speed_steer,-3.0f,3.0f);
    int left=(int)(base-speed_steer),right=(int)(base+speed_steer);
    if(state==DRIVE){
        /* Preserve requested left/right difference at the PWM ceiling.
           Independent clipping would halve steering authority at base=100. */
        int excess=(left>right?left:right)-DRIVE_FAST_PWM;
        if(excess>0){left-=excess;right-=excess;}
        left=ILimit(left,0,DRIVE_FAST_PWM);right=ILimit(right,0,DRIVE_FAST_PWM);
    }
    Motor_Set(left,right);
}

static int CornerDirection(uint32_t now,bool allow_tie){
    bool l=Ultrasonic_RecentGood(LEFT,now,RANGE_FRESH_MS)&&range[LEFT].fails<=2U;
    bool r=Ultrasonic_RecentGood(RIGHT,now,RANGE_FRESH_MS)&&range[RIGHT].fails<=2U;
    float L=range[LEFT].last_good,R=range[RIGHT].last_good;
    bool lg=MeasuredOpening(LEFT,now),rg=MeasuredOpening(RIGHT,now);
    /* This must precede absolute L/R comparison: an off-center car may
       still read less distance toward the opening than toward the opposite wall. */
    if(lg&&!rg){direction_reason="LEFT_MEASURED_GROWTH";return -1;}
    if(rg&&!lg){direction_reason="RIGHT_MEASURED_GROWTH";return 1;}
    if(lg&&rg){
        float a=L/fmaxf(range[LEFT].approach_wall,1.0f);
        float b=R/fmaxf(range[RIGHT].approach_wall,1.0f);
        if(a>1.3f*b){direction_reason="LEFT_STRONGER_GROWTH";return -1;}
        if(b>1.3f*a){direction_reason="RIGHT_STRONGER_GROWTH";return 1;}
    }
    bool lp=Ultrasonic_LatestGood(LEFT,now)&&range[LEFT].gap_votes&&!lg;
    bool rp=Ultrasonic_LatestGood(RIGHT,now)&&range[RIGHT].gap_votes&&!rg;
    if(lp||rp){
        direction_reason=lp?(rp?"VERIFY_BOTH_GROWTH":"VERIFY_LEFT_GROWTH"):
                           "VERIFY_RIGHT_GROWTH";
        return 0;
    }
    if(l&&r){
        if(range[LEFT].rising_count||range[RIGHT].rising_count){
            direction_reason="VERIFY_SIDE_CHANGE";return 0;
        }
        if(fabsf(L-R)>fmaxf(4.0f,0.08f*(L+R))){
            direction_reason=L>R?"LEFT_WIDER":"RIGHT_WIDER";return L>R?-1:1;
        }
        if(!allow_tie||!Ultrasonic_LatestGood(LEFT,now)||!Ultrasonic_LatestGood(RIGHT,now)){
            direction_reason="WAIT_OPENING";return 0;
        }
        direction_reason="BALANCED_PROBE";
        return L>=R?-1:1;
    }
    bool lh=range[LEFT].have_opening&&now-range[LEFT].opening_at<=OPENING_HINT_MS;
    bool rh=range[RIGHT].have_opening&&now-range[RIGHT].opening_at<=OPENING_HINT_MS;
    if(lh&&!rh&&r){direction_reason="LEFT_GAP_HINT";return -1;}
    if(rh&&!lh&&l){direction_reason="RIGHT_GAP_HINT";return 1;}
    /* Missing is not open: retain recent wall evidence and prefer a measured gap. */
    if(l&&!r&&L>=FRONT_CLEAR_CM&&range[RIGHT].have_good&&L>1.5f*R+5.0f){
        direction_reason="MEASURED_LEFT_SPACE";return -1;
    }
    if(r&&!l&&R>=FRONT_CLEAR_CM&&range[LEFT].have_good&&R>1.5f*L+5.0f){
        direction_reason="MEASURED_RIGHT_SPACE";return 1;
    }
    /* A near wall alone tells us how to center, not which way the route turns. */
    if((l&&!r&&L<=SIDE_TIGHT_TURN_CM)||(r&&!l&&R<=SIDE_TIGHT_TURN_CM)){
        direction_reason="NEED_OPEN_SIDE";return 0;
    }
    direction_reason="SIDES_UNKNOWN";
    return 0;
}

static void StartCorner(uint32_t now){
    state=CORNER;state_at=now;turn_phase=TURN_POWER;stalled_pivot=false;
    arc_active=false;arc_run_ms=0;
    if(pending_corner_direction){
        turn_direction=pending_corner_direction;direction_reason="RESUME_CORNER";
    }else{
        turn_direction=CornerDirection(now,range[FRONT].raw<=TIGHT_TURN_CM);
        RememberCorner(turn_direction,now);
    }
    turn_front_sequence=range[FRONT].sequence;turn_clear_count=0;
    turn_on_elapsed=0;turn_tick=now;turn_progress_at=now;turn_best=range[FRONT].raw;
    ResetCenter();reason="CORNER";
}

static bool SteeringApplied(void){
    int l=Motor_GetActual(0),r=Motor_GetActual(1);
    return turn_direction<0?l<r:l>r;
}

static bool RotationRoom(uint32_t now){
    return Ultrasonic_LatestGood(FRONT,now)&&range[FRONT].raw>FRONT_BACK_CM&&
           Ultrasonic_LatestGood(LEFT,now)&&Ultrasonic_LatestGood(RIGHT,now)&&
           range[LEFT].raw>=ROTATE_SIDE_CM&&range[RIGHT].raw>=ROTATE_SIDE_CM;
}

static bool TryCloseRotation(uint32_t now){
    if(!RotationRoom(now))return false;
    if(state!=CORNER){
        int d=pending_corner_direction?pending_corner_direction:CornerDirection(now,true);
        if(!d)return false;
        StartCorner(now);turn_direction=d;RememberCorner(d,now);
    }else if(!turn_direction){
        turn_direction=pending_corner_direction?pending_corner_direction:CornerDirection(now,true);
        if(!turn_direction)return false;
        RememberCorner(turn_direction,now);
    }
    if(!close_rotation_active){
        close_rotation_active=true;close_rotation_ms=0;close_rotation_tick=now;
        Motor_Stop(now);turn_phase=TURN_POWER;turn_on_elapsed=0;turn_tick=now;
    }
    uint32_t dt=now-close_rotation_tick;close_rotation_tick=now;
    int l=Motor_GetActual(0),r=Motor_GetActual(1);
    if((l<0&&r>0)||(l>0&&r<0))
        close_rotation_ms+=(dt>2U*CONTROL_MS)?2U*CONTROL_MS:dt;
    return close_rotation_ms<CLOSE_ROTATE_MS;
}

static void CornerTask(uint32_t now){
    Range *f=&range[FRONT];
    uint32_t dt=now-turn_tick;turn_tick=now;
    bool valid=Ultrasonic_LatestGood(FRONT,now)&&now-f->finished<=TURN_FRONT_AGE_MS;
    if(arc_active&&Motor_GetActual(0)>0&&Motor_GetActual(1)>0&&SteeringApplied())
        arc_run_ms+=(dt>2U*CONTROL_MS)?2U*CONTROL_MS:dt;
    /* During a confirmed corner, one missing return can be specular reflection.
       Two completed no-target cycles release the turn. Default DRIVE is unchanged. */
    bool no_target_clear=Ultrasonic_NoTarget(FRONT,now)&&f->no_target_count>=2U&&
                         now-f->no_target_at<=TURN_FRONT_AGE_MS;
    bool open=no_target_clear||(valid&&f->raw>=CORNER_RELEASE_CM);
    if(valid&&f->raw>FRONT_EMERGENCY_CM+2.0f)close_rotation_active=false;
    if(Emergency(now)&&!TryCloseRotation(now)){StartBack(now,"NO_ROTATE_CLEARANCE");return;}
    if(f->sequence!=turn_front_sequence){
        turn_front_sequence=f->sequence;
        if(no_target_clear)turn_clear_count=CORNER_CONFIRM_SAMPLES;
        else if(open){if(turn_clear_count<CORNER_CONFIRM_SAMPLES)++turn_clear_count;}
        else if(f->code!=NO_FALL)turn_clear_count=0;
        if(valid&&f->raw>=turn_best+TURN_PROGRESS_CM){turn_best=f->raw;turn_progress_at=now;}
    }
    /* The FIRST open sample stops turning. Confirmation occurs while moving straight. */
    if(open){
        if(turn_phase!=TURN_FORWARD){
            /* Side distances from the rotation describe another heading.
               Reset centering and wait for new side samples before steering. */
            side_alignment_gate=true;side_alignment_after=now;ResetCenter();
            /* A forward arc or one-wheel turn flows directly into forward.
               Only a sign reversal requires the existing electrical dead time. */
            if(Motor_GetActual(0)<0||Motor_GetActual(1)<0){
                Motor_Stop(now);Motor_HoldUntil(now+REVERSE_DEAD_MS);
            }
        }
        turn_phase=TURN_FORWARD;turn_on_elapsed=0;arc_active=false;
        ForwardCommand(now,true);reason="CORNER_OPEN_ALIGN";
        if(turn_clear_count>=2U){
            state=DRIVE;state_at=now;previous_direction=turn_direction;
            after_turn_until=now+AFTER_TURN_MS;ResetCenter();
            f->close_count=0;reason="CORNER_EXIT";
        }
        return;
    }
    if(!valid){
        Motor_Stop(now);
        arc_active=false;
        reason=Ultrasonic_LatestGood(FRONT,now)?"TURN_FRONT_STALE":
               Ultrasonic_NoTarget(FRONT,now)?"CORNER_VERIFY_NO_TARGET":"CORNER_ECHO_WAIT";
        if(!Ultrasonic_RecentGood(FRONT,now,FRONT_HOLD_MS))EnterAcquire(now,"CORNER_ECHO_LOST");
        return;
    }
    if(turn_phase==TURN_FORWARD){
        if(f->raw>FRONT_TURN_CM){ForwardCommand(now,true);reason="CORNER_ALIGN";return;}
        if(f->close_count<CORNER_CONFIRM_SAMPLES){
            Motor_Stop(now);reason="CORNER_RECHECK";return;
        }
        turn_phase=TURN_POWER;turn_on_elapsed=0;
    }
    if(!turn_direction){
        if(pending_corner_direction){
            turn_direction=pending_corner_direction;direction_reason="RESUME_CORNER";
        }else{
            turn_direction=CornerDirection(now,f->raw<=TIGHT_TURN_CM);
            RememberCorner(turn_direction,now);
        }
        if(!turn_direction){
            if(f->raw>TIGHT_TURN_CM){ForwardCommand(now,true);reason="APPROACH_OPENING";}
            else {StartBack(now,"CORNER_DIRECTION_UNKNOWN");}
            return;
        }
    }
    if(now-turn_progress_at>=TURN_STALL_MS){
        if(!stalled_pivot&&RotationRoom(now)){
            /* A square corner can stall a one-wheel turn: tighten the radius
               once before using a straight reverse. Preserve the chosen side. */
            stalled_pivot=true;turn_progress_at=now;turn_best=f->raw;
            Motor_Stop(now);turn_phase=TURN_POWER;turn_on_elapsed=0;
        }else {StartBack(now,"CORNER_NO_PROGRESS");return;}
    }
    /* No periodic stop/settle cycle. Fresh front returns bound continued
       motion; a stale or invalid front observation stops the turn above. */
    /* Most bends: hold one side, drive the other. Counterrotation is reserved for tight space. */
    SensorId outer_side=turn_direction<0?RIGHT:LEFT;
    bool outer_near=Ultrasonic_RecentGood(outer_side,now,RANGE_FRESH_MS)&&
                    range[outer_side].last_good<=SIDE_TIGHT_TURN_CM;
    SensorId inside_side=turn_direction<0?LEFT:RIGHT;
    bool roll= !stalled_pivot&&!Emergency(now)&&f->raw>=ARC_FRONT_CM&&
        arc_run_ms<ARC_MAX_RUN_MS&&
        Ultrasonic_LatestGood(inside_side,now)&&Ultrasonic_LatestGood(outer_side,now)&&
        now-range[inside_side].finished<=FAST_SIDE_AGE_MS&&
        now-range[outer_side].finished<=FAST_SIDE_AGE_MS&&
        range[inside_side].raw>=ARC_INNER_SPACE_CM&&
        range[outer_side].raw>=ARC_OUTER_SPACE_CM&&
        range[inside_side].rising_count==0U&&range[outer_side].rising_count==0U&&
        range[outer_side].closing_cmps<=FAST_CLOSING_CMPS;
    bool pivot=f->raw<=TIGHT_TURN_CM||outer_near||stalled_pivot||Emergency(now);
    int inner=pivot?-TURN_PWM:(roll?ARC_INNER_PWM:0);
    int outer=pivot?TURN_PWM:(roll?ARC_OUTER_PWM:CORNER_OUTER_PWM);
    arc_active=roll&&!pivot;
    if(turn_direction<0)Motor_Set(inner,outer);else Motor_Set(outer,inner);
    reason=Emergency(now)?"CLOSE_PIVOT":stalled_pivot?"STALL_PIVOT":
           pivot?"TIGHT_TURN":arc_active?"ROLLING_CORNER":"SQUARE_CORNER";
}

static void ControlTask(uint32_t now){
    if(state==STOP)return;
    if(state==MANUAL_DRIVE){Motor_Set(manual_left,manual_right);return;}
    if(state==TEST){
        if(now-state_at>=TEST_RUN_MS){Stop(now);reason="TEST_DONE";return;}
        if(test_physical>=0){Motor_TestPhysical((unsigned)test_physical,TEST_PWM);}
        else Motor_Set(test_left,test_right);
        return;
    }
    TrackCornerExit(now);
    if(state==BACK){
        uint32_t dt=now-turn_tick;turn_tick=now;
        if(Motor_GetActual(0)<0&&Motor_GetActual(1)<0)turn_on_elapsed+=dt;
        if(turn_on_elapsed>=BACK_RUN_MS){EnterAcquire(now,"BACK_REMEASURE");return;}
        Motor_Set(-BACK_PWM,-BACK_PWM);return;
    }
    bool any=Ultrasonic_RecentObservation(FRONT,now)||Ultrasonic_RecentObservation(LEFT,now)||Ultrasonic_RecentObservation(RIGHT,now);
    if(!any){
        if(state!=ACQUIRE)EnterAcquire(now,"ALL_INVALID");
        Motor_Stop(now);reason="ALL_INVALID";return;
    }
    if(state==CORNER){CornerTask(now);return;}
    if(state==ACQUIRE){
        Motor_Set(0,0);
        if(!Reached(now,acquire_after)||!Reached(range[FRONT].started,acquire_after))return;
        if(Ultrasonic_NoTarget(FRONT,now)){
            state=DRIVE;state_at=now;ResetCenter();ForwardCommand(now,true);
            reason="NO_TARGET_FORWARD";return;
        }
        if(Ultrasonic_LatestGood(FRONT,now)){
            if(Emergency(now)){
                if(TryCloseRotation(now))CornerTask(now);else StartBack(now,"NO_ROTATE_CLEARANCE");
                return;
            }
            if(range[FRONT].raw<=FRONT_TURN_CM){
                if(range[FRONT].close_count>=CORNER_CONFIRM_SAMPLES){StartCorner(now);CornerTask(now);}
                return;
            }
            state=DRIVE;state_at=now;ResetCenter();ForwardCommand(now,true);
            reason="SENSE_READY";return;
        }
        /* Only electrical/pulse faults reach this wait. A completed no-echo cycle
           takes the default-forward path above; missing range never commands BACK. */
        reason="FRONT_SIGNAL_FAULT";return;
    }
    if(Emergency(now)){
        if(TryCloseRotation(now))CornerTask(now);else StartBack(now,"NO_ROTATE_CLEARANCE");
        return;
    }
    /* Default action: forward. A completed no-target cycle is not a wall. */
    if(Ultrasonic_NoTarget(FRONT,now)){
        ForwardCommand(now,true);reason="NO_TARGET_FORWARD";return;
    }
    if(!Ultrasonic_LatestGood(FRONT,now)){
        if(Ultrasonic_RecentGood(FRONT,now,FRONT_HOLD_MS)&&range[FRONT].last_good>=FRONT_SLOW_CM&&range[FRONT].fails<=2U){
            ForwardCommand(now,true);reason="SHORT_ECHO_GAP";return;
        }
        EnterAcquire(now,"FRONT_REMEASURE");return;
    }
    if(range[FRONT].raw<=FRONT_TURN_CM&&range[FRONT].close_count>=CORNER_CONFIRM_SAMPLES){
        StartCorner(now);CornerTask(now);return;
    }
    ForwardCommand(now,false);reason="WALL_FOLLOW";
}

static void ButtonTask(uint32_t now){
    GPIO_PinState x=HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13);
    if(x!=button_raw){button_raw=x;button_at=now;}
    if(x==button_stable||now-button_at<BUTTON_DEBOUNCE_MS)return;
    button_stable=x;if(x==button_idle)return;
    ToggleMode(now);
}

void App_DebugCommand(char c,uint32_t now){
    if(c>='a'&&c<='z')c=(char)(c-'a'+'A');
    if(c=='S'){Stop(now);return;}
    if(c=='G'){if(state==STOP)Start(now);return;}
    /* Test requests only work from STOP, so noise cannot change autonomous behavior. */
    if(state!=STOP)return;
    test_physical=-1;test_left=0;test_right=0;
    if(c=='M')test_physical=0;
    else if(c=='N')test_physical=1;
    else if(c=='L')test_right=TEST_PWM;
    else if(c=='R')test_left=TEST_PWM;
    else return;
    Motor_Stop(now);state=TEST;state_at=now;reason="MOTOR_MAPPING_TEST";
}

static void LogTask(uint32_t now){
    if(now-last_log<LOG_MS||!Ultrasonic_IsIdle())return;
    last_log=now;
    if(DebugUart_IsBusy()){++log_overruns;return;}
    static const char *const states[]={"STOP","SENSE","DRIVE","CORNER","BACK","TEST","BT_DRIVE"};
    static const char *const codes[]={"EMPTY","OK","NO_RISE","NO_FALL","HIGH","CLOSE","GLITCH","NO_TARGET"};
    char *tx=DebugUart_Buffer();
    int n=snprintf(tx,DEBUG_UART_BUFFER_SIZE,
      "%s %lu %s/%s F=%d/%s age=%lu L=%d/%s R=%d/%s cmd=%d,%d dir=%d/%s lock=%d run=%lu gap=%u,%u %s\r\n",
      APP_VERSION,(unsigned long)now,control_mode==MODE_AUTO?"AUTO":"MANUAL",states[state],
      Ultrasonic_LatestGood(FRONT,now)?(int)range[FRONT].raw:-1,codes[range[FRONT].code],
      (unsigned long)(range[FRONT].have_good?now-range[FRONT].good_at:0),
      Ultrasonic_LatestGood(LEFT,now)?(int)range[LEFT].raw:-1,codes[range[LEFT].code],
      Ultrasonic_LatestGood(RIGHT,now)?(int)range[RIGHT].raw:-1,codes[range[RIGHT].code],
      Motor_GetActual(0),Motor_GetActual(1),turn_direction,direction_reason,
      pending_corner_direction,(unsigned long)corner_forward_ms,
      (unsigned)MeasuredOpening(LEFT,now),(unsigned)MeasuredOpening(RIGHT,now),reason);
    if(n>0&&n<(int)DEBUG_UART_BUFFER_SIZE)DebugUart_Commit((unsigned)n);
}

bool App_SensorFrontPriority(void){return state==ACQUIRE||state==CORNER;}
bool App_SideHistoryIsFresh(uint32_t good_at){
    return !side_alignment_gate||Reached(good_at,side_alignment_after);
}
bool App_IsManual(void){return control_mode==MODE_MANUAL;}
void App_StopWithReason(uint32_t now,const char *why){Stop(now);reason=why;}
void App_ManualDrive(int l,int r,uint32_t now){
    if(control_mode!=MODE_MANUAL)return;
    if(state!=MANUAL_DRIVE){Motor_Stop(now);Motor_CancelHold();}
    manual_left=l;manual_right=r;state=MANUAL_DRIVE;state_at=now;
    Motor_Set(l,r);reason="BT_DRIVE";
}
void App_Init(void){
    Board_Init();DebugUart_Init();Motor_Init();Ultrasonic_Init();
    button_idle=HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13);button_raw=button_idle;button_stable=button_idle;
    Motor_Stop(HAL_GetTick());
}
void App_Task(void){
    uint32_t now=HAL_GetTick();Ultrasonic_Task(now);
    if(Ultrasonic_IsTriggerActive())return;
    ButtonTask(now);BT_Poll(now);DebugUart_Poll(now);
    if(now-last_control>=CONTROL_MS){last_control=now;ControlTask(now);}
    Motor_Task(now);LogTask(now);
    bool led=control_mode==MODE_MANUAL?(state!=STOP):
             (state!=ACQUIRE||(now/250U)%2U==0U);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,led?GPIO_PIN_SET:GPIO_PIN_RESET);
}
