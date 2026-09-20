#include <math.h>
#include <stdio.h>
#include <string.h>

#include "DFCom.h"
#include "nuedc_2026_routes.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    float vx;
    float vy;
    float vz;
    u32 tick_ms;
} VelocityCommand;

volatile OdomData_t g_odom;
volatile u32 g_local_tick_ms;
u8 g_dfcom_unit_mode;

static VelocityCommand g_commands[16];
static unsigned g_command_count;
static unsigned g_failures;
static u8 g_subscribe_mode;
static u16 g_subscribe_frequency;
static int g_frames_enabled;
static int g_motion_enabled;
static float g_current_vx;
static float g_current_vy;
static float g_current_vz;

#define CHECK(condition)                                                        \
    do {                                                                        \
        if (!(condition)) {                                                     \
            printf("FAIL %s:%d: %s\n", __func__, __LINE__, #condition);       \
            g_failures++;                                                       \
        }                                                                       \
    } while (0)

static int nearly_equal(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static float normalize_angle(float angle)
{
    while (angle > (float)M_PI) angle -= 2.0f * (float)M_PI;
    while (angle < -(float)M_PI) angle += 2.0f * (float)M_PI;
    return angle;
}

static void reset_simulation(float initial_yaw)
{
    memset((void *)&g_odom, 0, sizeof(g_odom));
    memset(g_commands, 0, sizeof(g_commands));
    g_odom.yaw_rad = initial_yaw;
    g_local_tick_ms = 0;
    g_dfcom_unit_mode = DFCOM_UNIT_CM;
    g_command_count = 0;
    g_subscribe_mode = 0;
    g_subscribe_frequency = 0;
    g_frames_enabled = 1;
    g_motion_enabled = 1;
    g_current_vx = 0.0f;
    g_current_vy = 0.0f;
    g_current_vz = 0.0f;
}

void Cmd_Subscribe_Odom(u8 mode, u16 freq_hz)
{
    g_subscribe_mode = mode;
    g_subscribe_frequency = freq_hz;
}

void Cmd_Move_Vel(float vx_mps, float vy_mps, float vz_rad_s)
{
    CHECK(g_command_count < (sizeof(g_commands) / sizeof(g_commands[0])));
    if (g_command_count < (sizeof(g_commands) / sizeof(g_commands[0]))) {
        g_commands[g_command_count].vx = vx_mps;
        g_commands[g_command_count].vy = vy_mps;
        g_commands[g_command_count].vz = vz_rad_s;
        g_commands[g_command_count].tick_ms = g_local_tick_ms;
        g_command_count++;
    }
    g_current_vx = vx_mps;
    g_current_vy = vy_mps;
    g_current_vz = vz_rad_s;
}

void delay_ms(u32 ms)
{
    float dt = (float)ms / 1000.0f;
    float mid_yaw = g_odom.yaw_rad + 0.5f * g_current_vz * dt;

    g_local_tick_ms += ms;
    if (!g_frames_enabled) return;

    if (g_motion_enabled) {
        g_odom.n_pos_x_m += g_current_vx * cosf(mid_yaw) * dt
                          - g_current_vy * sinf(mid_yaw) * dt;
        g_odom.n_pos_y_m += g_current_vx * sinf(mid_yaw) * dt
                          + g_current_vy * cosf(mid_yaw) * dt;
        g_odom.yaw_rad = normalize_angle(g_odom.yaw_rad + g_current_vz * dt);
    }
    g_odom.frame_count++;
}

static void check_common_success_contract(
    Nuedc2026Status status,
    float expected_speed,
    float expected_radius,
    u32 maximum_time_ms)
{
    unsigned index;
    float expected_omega = -expected_speed / expected_radius;

    CHECK(status == NUEDC_2026_STATUS_OK);
    CHECK(g_dfcom_unit_mode == DFCOM_UNIT_M);
    CHECK(g_subscribe_mode == ODOM_MODE_CONTINUOUS);
    CHECK(g_subscribe_frequency == ODOM_FREQ_100HZ);
    CHECK(g_command_count == 5U);

    for (index = 0; index < 4U && index < g_command_count; index++) {
        CHECK(g_commands[index].vx > 0.0f);
        CHECK(nearly_equal(g_commands[index].vy, 0.0f, 0.0001f));
    }

    CHECK(nearly_equal(g_commands[0].vx, expected_speed, 0.0001f));
    CHECK(nearly_equal(g_commands[0].vz, 0.0f, 0.0001f));
    CHECK(nearly_equal(g_commands[1].vx, expected_speed, 0.0001f));
    CHECK(nearly_equal(g_commands[1].vz, expected_omega, 0.0002f));
    CHECK(nearly_equal(g_commands[2].vx, expected_speed, 0.0001f));
    CHECK(nearly_equal(g_commands[2].vz, 0.0f, 0.0001f));
    CHECK(nearly_equal(g_commands[3].vx, expected_speed, 0.0001f));
    CHECK(nearly_equal(g_commands[3].vz, expected_omega, 0.0002f));
    CHECK(nearly_equal(g_commands[4].vx, 0.0f, 0.0001f));
    CHECK(nearly_equal(g_commands[4].vy, 0.0f, 0.0001f));
    CHECK(nearly_equal(g_commands[4].vz, 0.0f, 0.0001f));
    CHECK(g_local_tick_ms < maximum_time_ms);
    CHECK(hypotf(g_odom.n_pos_x_m, g_odom.n_pos_y_m) < 0.04f);
}

static void test_h_route_is_continuous_and_under_20_seconds(void)
{
    Nuedc2026Status status;

    reset_simulation(3.0f); /* 第一段就跨越 +pi/-pi，验证航向展开。 */
    status = Nuedc2026_RunHRoute();

    check_common_success_contract(
        status,
        NUEDC_2026_H_SPEED_MPS,
        NUEDC_2026_H_RADIUS_M,
        20000U);
}

static void test_default_speeds_are_moderate(void)
{
    CHECK(nearly_equal(NUEDC_2026_H_SPEED_MPS, 0.34f, 0.0001f));
    CHECK(nearly_equal(NUEDC_2026_D_SPEED_MPS, 0.25f, 0.0001f));
}

static void test_d_route_is_continuous_and_under_90_seconds(void)
{
    Nuedc2026Status status;

    reset_simulation(-2.8f);
    status = Nuedc2026_RunDRoute();

    check_common_success_contract(
        status,
        NUEDC_2026_D_SPEED_MPS,
        NUEDC_2026_D_RADIUS_M,
        90000U);
}

static void test_missing_odometry_stops_safely(void)
{
    Nuedc2026Status status;

    reset_simulation(0.0f);
    g_frames_enabled = 0;
    status = Nuedc2026_RunHRoute();

    CHECK(status == NUEDC_2026_STATUS_ODOM_STALE);
    CHECK(g_command_count == 1U);
    CHECK(nearly_equal(g_commands[0].vx, 0.0f, 0.0001f));
    CHECK(g_local_tick_ms <= NUEDC_2026_ODOM_START_TIMEOUT_MS
                           + NUEDC_2026_CONTROL_PERIOD_MS);
}

static void test_motion_timeout_stops_safely(void)
{
    Nuedc2026Status status;

    reset_simulation(0.0f);
    g_motion_enabled = 0;
    status = Nuedc2026_RunHRoute();

    CHECK(status == NUEDC_2026_STATUS_SEGMENT_TIMEOUT);
    CHECK(g_command_count == 2U);
    CHECK(g_commands[0].vx > 0.0f);
    CHECK(nearly_equal(g_commands[1].vx, 0.0f, 0.0001f));
}

static void test_invalid_route_stops_without_starting(void)
{
    Nuedc2026Status status;

    reset_simulation(0.0f);
    status = Nuedc2026_RunRoute((Nuedc2026RouteId)99);

    CHECK(status == NUEDC_2026_STATUS_INVALID_ROUTE);
    CHECK(g_subscribe_frequency == 0U);
    CHECK(g_command_count == 1U);
    CHECK(nearly_equal(g_commands[0].vx, 0.0f, 0.0001f));
}

int main(void)
{
    test_default_speeds_are_moderate();
    test_h_route_is_continuous_and_under_20_seconds();
    test_d_route_is_continuous_and_under_90_seconds();
    test_missing_odometry_stops_safely();
    test_motion_timeout_stops_safely();
    test_invalid_route_stops_without_starting();

    if (g_failures != 0U) {
        printf("%u NUEDC 2026 route test(s) failed\n", g_failures);
        return 1;
    }

    printf("all NUEDC 2026 H/D fixed-route tests passed\n");
    return 0;
}
