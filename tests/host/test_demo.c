/* Exercise actual main with missing telemetry and independent queue boundaries. */
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "DFCom.h"
int demo_main(void);
volatile u32 g_local_tick_ms;
volatile OdomData_t g_odom;
volatile DcarState_t g_dcar_state;
volatile u8 g_print_odom_enable;
static jmp_buf finished;
static const char *scenario;
static unsigned submitted, waited, begun, canceled, stops, sessions, odom_subs, polls;
void DFCom_Poll(void) { polls++; }
void delay_init(void) {}
void uart_init(u32 baud) { assert(baud == 460800); }
void usart2_init(u32 baud) { assert(baud == 115200); }
void Odom_PrintTimer_Init(void) {}
void DFCom_MoveSessionQuarantine(void) { assert(sessions == 0); sessions++; }
void DFCom_MoveSessionStart(void) { assert(stops == 2); sessions++; }
void Cmd_Move_Vel(float x,float y,float z) { assert(x==0 && y==0 && z==0); stops++; }
void delay_ms(u32 ms) {
    if (canceled) longjmp(finished,1);
    g_local_tick_ms += ms;
}
void Cmd_Subscribe_Odom(u8 mode,u16 hz) {
    assert(mode==ODOM_MODE_CONTINUOUS && hz==ODOM_FREQ_10HZ); odom_subs++;
    if (strcmp(scenario,"with-odom")==0 || strcmp(scenario,"odom-only")==0) g_odom.frame_count++;
}
void Cmd_Query_DcarState(void) {
    if (strcmp(scenario,"no-state")!=0) g_dcar_state.received=1;
    g_dcar_state.imu_calibrated=strcmp(scenario,"uncalibrated")!=0;
}
u8 DFQueue_Begin(void) {
    assert(g_local_tick_ms==4570); /* 1000 + 20 + 500 + 50 + 3000: delays preserved. */
    assert(polls>=914);
    if (begun==3) longjmp(finished,1);
    begun++;
    return strcmp(scenario,"begin-failure")!=0;
}
static u16 task(void) {
    submitted++;
    return strcmp(scenario,"submit-failure")==0 ? 0 : (u16)submitted;
}
u16 DFQueue_Linear(float x,float y,float speed,float yaw,u8 grade) {
    assert(grade==DF_ACCEL_MEDIUM && y==0 && x==-.60f && speed==.15f);
    assert((submitted%4==1 && yaw==0) || (submitted%4==3 && yaw>6.28f));
    return task();
}
u16 DFQueue_Rotate(float yaw,float omega) {
    (void)yaw; (void)omega; assert(!"no standalone rotate"); return task();
}
u16 DFQueue_Arc(float radius,float yaw,float speed,u8 grade) {
    assert(submitted%2==0 && radius==.30f && yaw>3.14f && speed==-.15f && grade==DF_ACCEL_MEDIUM);
    return task();
}
u8 DFQueue_Wait(u16 id,u32 timeout) {
    waited++; assert(id==waited && timeout==60000);
    if (strcmp(scenario,"wait-warning")==0) return DF_QUEUE_WAIT_WARN;
    return strcmp(scenario,"wait-failure")==0 ? DF_QUEUE_WAIT_FAULT : DF_QUEUE_WAIT_DONE;
}
void DFQueue_Cancel(void) { canceled++; }
const char *DFQueue_LastError(void) { return "simulated boundary failure"; }
u8 DFQueue_LastNotice(void) { return 0x26; }
int main(int argc,char **argv) {
    assert(argc==2); scenario=argv[1];
    if (!setjmp(finished)) demo_main();
    assert(stops==2 && sessions==2 && odom_subs==1);
    if (strcmp(scenario,"begin-failure")==0)
        assert(begun==1 && submitted==0 && waited==0 && canceled);
    else if (strcmp(scenario,"submit-failure")==0)
        assert(submitted==1 && waited==0 && canceled);
    else if (strcmp(scenario,"wait-failure")==0)
        assert(submitted==4 && waited==1 && canceled);
    else
        assert(begun==3 && submitted==12 && waited==12 && !canceled);
    puts("PASS: full startup delays, independent telemetry, three laps and queue failure handling");
    return 0;
}
