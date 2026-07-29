#ifndef TEST_NUEDC_DFCOM_H
#define TEST_NUEDC_DFCOM_H

#include <stdint.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

#define ODOM_MODE_CONTINUOUS 0x01U
#define ODOM_FREQ_100HZ      100U

typedef enum {
    DFCOM_UNIT_CM = 0,
    DFCOM_UNIT_M = 1,
    DFCOM_UNIT_MM = 2
} DFCom_UnitMode_e;

typedef struct {
    float yaw_rad;
    float n_pos_x_m;
    float n_pos_y_m;
    u32 frame_count;
} OdomData_t;

extern volatile OdomData_t g_odom;
extern volatile u32 g_local_tick_ms;
extern u8 g_dfcom_unit_mode;

void Cmd_Subscribe_Odom(u8 mode, u16 freq_hz);
void Cmd_Move_Vel(float vx_mps, float vy_mps, float vz_rad_s);

#endif
