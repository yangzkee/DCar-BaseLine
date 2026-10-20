#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "DFCom.h"
volatile u32 g_local_tick_ms;
void DFCom_Poll(void) {}
static u8 tx[100],txlen;
static u8 no_reply,drop_ack,tx_error,split_at;
static u16 capability=6;
static u8 available=16;
static u16 next_id=65535;
static void make_reply(u8 *b,u8 progress,u8 notice,u16 id){
    u16 sum=0;u8 i;
    b[0]=0xdf;b[1]=DFCOM_PC_ID;b[2]=DFCOM_ROBOT_ID;b[3]=0x6f;b[4]=0x85;b[5]=4;
    b[6]=progress;b[7]=notice;b[8]=(u8)id;b[9]=(u8)(id>>8);b[10]=0xfd;
    for(i=0;i<11;i++)sum+=b[i];
    b[11]=(u8)sum;b[12]=(u8)(sum>>8);
}
static void reply(u8 progress,u8 notice,u16 id){
    u8 b[13];make_reply(b,progress,notice,id);
    if(split_at){ DFCom_RxParse(b,split_at); DFCom_RxParse(b+split_at,13-split_at); }
    else DFCom_RxParse(b,13);
}
void delay_ms(u32 ms){g_local_tick_ms+=ms;}
void delay_us(u32 us){(void)us;}
u8 USART1_Send_By_DMA(u8 *b,u8 len){
    u16 id;memcpy(tx,b,len);txlen=len;
    if(tx_error)return 1;
    if(no_reply)return 0;
    if(b[4]==0x85){
        if(b[6]==0)reply(16,0x21,capability);
        if(b[6]==7)reply(available,0x28,next_id);
        if(b[6]==5)reply(255,0,(u16)b[7]|((u16)b[8]<<8));
    }else if(b[4]>=0x63 && b[4]<=0x66){
        id=(u16)b[4+b[5]]|((u16)b[5+b[5]]<<8);
        if(drop_ack){drop_ack=0;return 0;}
        reply(0,0x10,id);
    }
    return 0;
}
static void telemetry_reply(u8 b, u8 len, u8 ver) {
    u8 frame[69] = {0xdf,DFCOM_PC_ID,DFCOM_ROBOT_ID,0x6c,0,0,0};
    u16 sum = 0;
    unsigned i;
    frame[4]=b; frame[5]=len; frame[6]=ver; frame[6+len]=0xfd;
    for(i=0;i<(unsigned)len+7;i++) sum+=frame[i];
    frame[len+7]=(u8)sum; frame[len+8]=(u8)(sum>>8);
    DFCom_RxParse(frame,(u16)(len+9));
}
int main(void){
    u16 a,b,c,sum;u8 p,n,i;
    u8 joined[26];
    u8 telemetry[44]={0xdf,DFCOM_PC_ID,DFCOM_ROBOT_ID,0x6c,0x81,35,2};
    u8 bad_length[6]={0xdf,DFCOM_PC_ID,DFCOM_ROBOT_ID,0x6f,0x85,60};
    assert(DFQueue_Begin());drop_ack=1;
    a=DFQueue_Linear(.1f,0,.06f,0,18);assert(a==65535 && tx[4]==0x64 && tx[5]==15 && txlen==24);
    b=DFQueue_Linear(-.6f,0,.15f,6.2831853f,18);assert(b==1 && tx[4]==0x64 && tx[5]==19);
    reply(255,0,a);reply(255,0,a);assert(DFQueue_Status(b,&p,&n) && p==0);
    reply(254,0x25,b);assert(DFQueue_Status(b,&p,&n) && p==254);
    reply(255,0x27,b);reply(2,0,b);assert(DFQueue_Status(b,&p,&n) && p==255 && n==0x27);
    assert(DFQueue_Wait(a,1000)==1);assert(DFQueue_Wait(b,1000)==1);
    c=DFQueue_Arc(.3f,3.14f,-.15f,18);assert(c==2 && tx[4]==0x66 && tx[5]==15);
    assert(DFQueue_Wait(c,1000)==1); /* lost completion recovered via query */
    assert(!DFQueue_Linear(1,0,-.1f,0,18));assert(!DFQueue_Linear(1,0,.1f,0,2));
    c=DFQueue_Rotate(1,.5f);assert(c==3 && tx[5]==10);
    no_reply=1;assert(DFQueue_Wait(c,20)==2);assert(tx[4]==0x62);
    assert(!DFQueue_Linear(.1f,0,.1f,0,18));
    assert(strstr(DFQueue_LastError(), "Begin"));
    no_reply=0;
    capability=5;assert(!DFQueue_Begin());assert(strstr(DFQueue_LastError(),"v6"));capability=6;
    available=15;assert(!DFQueue_Begin());assert(strstr(DFQueue_LastError(),"not empty"));available=16;
    tx_error=1;assert(!DFQueue_Begin());assert(strstr(DFQueue_LastError(),"DMA"));tx_error=0;
    no_reply=1;assert(!DFQueue_Begin());assert(strstr(DFQueue_LastError(),"timed out"));no_reply=0;
    /* Every split inside a queue reply must survive across separate RX calls. */
    for(i=1;i<13;i++){
        DFQueue_Cancel();split_at=i;assert(DFQueue_Begin());
        c=DFQueue_Linear(.1f,0,.1f,0,18);assert(c);
        reply(255,0,c);assert(DFQueue_Wait(c,100)==DF_QUEUE_WAIT_DONE);
    }
    split_at=0;assert(DFQueue_Begin());
    c=DFQueue_Linear(.1f,0,.1f,0,18);assert(c);
    /* A corrupt but plausible LEN must not hide the valid terminal following it. */
    DFCom_RxParse(bad_length,sizeof(bad_length));reply(255,0,c);
    assert(DFQueue_Wait(c,100)==DF_QUEUE_WAIT_DONE);
    c=DFQueue_Rotate(1,.5f);assert(c);reply(255,0x26,c);
    assert(DFQueue_Wait(c,100)==DF_QUEUE_WAIT_FAULT);
    assert(DFQueue_LastNotice()==0x26 && strstr(DFQueue_LastError(),"failure"));
    assert(DFQueue_Begin());c=DFQueue_Rotate(1,.5f);assert(c);
    assert(DFQueue_Wait(0,100)==DF_QUEUE_WAIT_TIMEOUT);
    assert(DFQueue_Wait(c,0)==DF_QUEUE_WAIT_TIMEOUT);
    reply(255,0,c);assert(DFQueue_Wait(c,100)==DF_QUEUE_WAIT_DONE);
    c=DFQueue_Rotate(1,.5f);assert(c);tx_error=1;
    assert(!DFQueue_Rotate(1,.5f));assert(strstr(DFQueue_LastError(),"DMA"));tx_error=0;
    assert(DFQueue_Begin());a=DFQueue_Rotate(1,.5f);b=DFQueue_Rotate(1,.5f);assert(a && b);
    make_reply(joined,255,0,a);joined[11]^=1;
    DFCom_RxParse(joined,13);assert(DFQueue_Status(a,&p,&n) && p==0); /* bad checksum ignored */
    make_reply(joined,255,0,a);make_reply(joined+13,255,0,b);
    DFCom_RxParse(joined,sizeof(joined)); /* two terminal frames in one receive */
    assert(DFQueue_Wait(a,100)==DF_QUEUE_WAIT_DONE && DFQueue_Wait(b,100)==DF_QUEUE_WAIT_DONE);
    telemetry[41]=0xfd;sum=0;for(i=0;i<42;i++)sum+=telemetry[i];
    telemetry[42]=(u8)sum;telemetry[43]=(u8)(sum>>8);
    for(i=0;i<44;i++)DFCom_RxParse(telemetry+i,1);
    assert(g_odom.frame_count==0); /* removed 0x81 ignored */
    Cmd_Subscribe_Odom(ODOM_MODE_CONTINUOUS,10);
    assert(tx[3]==4 && tx[4]==0x80 && tx[5]==2 && tx[6]==1 && tx[7]==1);
    telemetry_reply(0x80,51,4);
    telemetry_reply(0x81,35,2);
    telemetry_reply(0x80,51,4);
    telemetry_reply(0x81,35,2);
    assert(g_odom.frame_count==2);
    assert(g_dfcom_rx_diag.odom_wire==2);
    telemetry_reply(0x81,35,3);
    telemetry_reply(0x81,0,0);
    telemetry_reply(0x80,50,4);
    assert(g_odom.frame_count==2);
    assert(g_dfcom_rx_diag.odom_rejected==1);
    puts("queue wire, rollover, duplicate, trim, query, failures and fragmented/coalesced RX passed");return 0;
}
