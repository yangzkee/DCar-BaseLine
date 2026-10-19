#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "DFCom.h"
volatile u32 g_local_tick_ms;
static u8 tx[100],txlen;
static u8 no_reply,drop_ack;
static u16 next_id=65535;
static void reply(u8 progress,u8 notice,u16 id){
    u8 b[13]={0xdf,DFCOM_PC_ID,DFCOM_ROBOT_ID,0x6f,0x85,4};u16 sum=0;u8 i;
    b[6]=progress;b[7]=notice;b[8]=(u8)id;b[9]=(u8)(id>>8);b[10]=0xfd;
    for(i=0;i<11;i++)sum+=b[i];
    b[11]=(u8)sum;b[12]=(u8)(sum>>8);DFCom_RxParse(b,13);
}
void delay_ms(u32 ms){g_local_tick_ms+=ms;}
void delay_us(u32 us){(void)us;}
u8 USART1_Send_By_DMA(u8 *b,u8 len){
    u16 id;memcpy(tx,b,len);txlen=len;
    if(no_reply)return 0;
    if(b[4]==0x85){
        if(b[6]==0)reply(16,0x21,6);
        if(b[6]==7)reply(16,0x28,next_id);
        if(b[6]==5)reply(255,0,(u16)b[7]|((u16)b[8]<<8));
    }else if(b[4]>=0x63 && b[4]<=0x66){
        id=(u16)b[4+b[5]]|((u16)b[5+b[5]]<<8);
        if(drop_ack){drop_ack=0;return 0;}
        reply(0,0x10,id);
    }
    return 0;
}
int main(void){
    u16 a,b,c;u8 p,n;
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
    puts("queue wire, rollover, duplicate, trim, query and timeout tests passed");return 0;
}
