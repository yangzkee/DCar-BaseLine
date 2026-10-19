#ifndef DF_BETA_EXAMPLES_H
#define DF_BETA_EXAMPLES_H
#include "DFCom.h"
/* Explicit opt-in: no automatic motion in an unconfigured freshly flashed sample. */
#define DF_BETA_RUN 0
#define DF_BETA_ROUTE 1 /* 1 stadium, 2 square with in-place 90 degree turns */
#define DF_BETA_SPEED 0.15f
#define DF_BETA_GRADE DF_ACCEL_MEDIUM
static u8 DF_BetaExample(void) {
    u16 ids[8];u8 i,n;
    const float pi=3.14159265358979323846f;
    if(!DFQueue_Begin())return 0;
    if(DF_BETA_ROUTE==1){
        n=4;
        ids[0]=DFQueue_Arc(.30f,pi,-DF_BETA_SPEED,DF_BETA_GRADE);if(!ids[0])goto fail;
        ids[1]=DFQueue_Linear(-.60f,0,DF_BETA_SPEED,0,DF_BETA_GRADE);if(!ids[1])goto fail;
        ids[2]=DFQueue_Arc(.30f,pi,-DF_BETA_SPEED,DF_BETA_GRADE);if(!ids[2])goto fail;
        /* Holonomic only: world path stays straight while yaw makes one full turn. */
        ids[3]=DFQueue_Linear(-.60f,0,DF_BETA_SPEED,2*pi,DF_BETA_GRADE);if(!ids[3])goto fail;
    }else if(DF_BETA_ROUTE==2){
        n=8;
        for(i=0;i<4;i++){
            ids[2*i]=DFQueue_Linear(.50f,0,DF_BETA_SPEED,0,DF_BETA_GRADE);if(!ids[2*i])goto fail;
            ids[2*i+1]=DFQueue_Rotate(pi/2,.5f);if(!ids[2*i+1])goto fail;
        }
    }else goto fail;
    for(i=0;i<n;i++)if(DFQueue_Wait(ids[i],60000)!=1)goto fail;
    return 1;
fail:
    DFQueue_Cancel();return 0;
}
#endif
