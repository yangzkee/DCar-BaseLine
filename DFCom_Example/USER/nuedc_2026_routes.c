#include "nuedc_2026_routes.h"

#include <math.h>

#include "DFCom.h"
#include "delay.h"

typedef struct {
    float x_m;
    float y_m;
    float yaw_rad;
    u32 frame_count;
} Nuedc2026Pose;

typedef struct {
    float radius_m;
    float speed_mps;
    float arc_omega_scale;
    u32 straight_timeout_ms;
    u32 arc_timeout_ms;
} Nuedc2026RouteConfig;

static float normalize_angle(float angle_rad)
{
    while (angle_rad > NUEDC_2026_HALF_TURN_RAD) {
        angle_rad -= 2.0f * NUEDC_2026_HALF_TURN_RAD;
    }
    while (angle_rad < -NUEDC_2026_HALF_TURN_RAD) {
        angle_rad += 2.0f * NUEDC_2026_HALF_TURN_RAD;
    }
    return angle_rad;
}

/* ODOM 在串口中断末尾才增加 frame_count；前后帧号一致即可得到同一帧快照。 */
static Nuedc2026Pose read_pose(void)
{
    Nuedc2026Pose pose;
    u32 frame_before;
    u32 frame_after;

    do {
        frame_before = g_odom.frame_count;
        pose.x_m = g_odom.n_pos_x_m;
        pose.y_m = g_odom.n_pos_y_m;
        pose.yaw_rad = g_odom.yaw_rad;
        frame_after = g_odom.frame_count;
    } while (frame_before != frame_after);

    pose.frame_count = frame_after;
    return pose;
}

static Nuedc2026Status stop_with_status(Nuedc2026Status status)
{
    Cmd_Move_Vel(0.0f, 0.0f, 0.0f);
    return status;
}

static Nuedc2026Status wait_for_live_odometry(void)
{
    Nuedc2026Pose initial = read_pose();
    u32 start_tick = g_local_tick_ms;

    while ((u32)(g_local_tick_ms - start_tick)
           <= NUEDC_2026_ODOM_START_TIMEOUT_MS) {
        Nuedc2026Pose current;

        delay_ms(NUEDC_2026_CONTROL_PERIOD_MS);
        current = read_pose();
        if (current.frame_count != initial.frame_count) {
            return NUEDC_2026_STATUS_OK;
        }
    }

    return NUEDC_2026_STATUS_ODOM_STALE;
}

static Nuedc2026Status run_straight(float speed_mps, u32 timeout_ms)
{
    Nuedc2026Pose start = read_pose();
    u32 segment_start_tick = g_local_tick_ms;
    u32 last_frame_tick = g_local_tick_ms;
    u32 last_frame_count = start.frame_count;
    float heading_cos = cosf(start.yaw_rad);
    float heading_sin = sinf(start.yaw_rad);

    Cmd_Move_Vel(speed_mps, 0.0f, 0.0f);

    for (;;) {
        Nuedc2026Pose current;
        u32 now;

        delay_ms(NUEDC_2026_CONTROL_PERIOD_MS);
        now = g_local_tick_ms;
        current = read_pose();

        if (current.frame_count != last_frame_count) {
            float dx = current.x_m - start.x_m;
            float dy = current.y_m - start.y_m;
            float forward_m = dx * heading_cos + dy * heading_sin;

            last_frame_count = current.frame_count;
            last_frame_tick = now;
            if (forward_m >= NUEDC_2026_STRAIGHT_LENGTH_M) {
                return NUEDC_2026_STATUS_OK;
            }
        }

        if ((u32)(now - last_frame_tick) >= NUEDC_2026_ODOM_STALE_TIMEOUT_MS) {
            return NUEDC_2026_STATUS_ODOM_STALE;
        }
        if ((u32)(now - segment_start_tick) >= timeout_ms) {
            return NUEDC_2026_STATUS_SEGMENT_TIMEOUT;
        }
    }
}

static Nuedc2026Status run_clockwise_half_arc(
    float radius_m,
    float speed_mps,
    float omega_scale,
    u32 timeout_ms)
{
    Nuedc2026Pose previous = read_pose();
    u32 segment_start_tick = g_local_tick_ms;
    u32 last_frame_tick = g_local_tick_ms;
    u32 last_frame_count = previous.frame_count;
    float accumulated_yaw = 0.0f;
    float clockwise_omega = -(speed_mps / radius_m) * omega_scale;

    Cmd_Move_Vel(speed_mps, 0.0f, clockwise_omega);

    for (;;) {
        Nuedc2026Pose current;
        u32 now;

        delay_ms(NUEDC_2026_CONTROL_PERIOD_MS);
        now = g_local_tick_ms;
        current = read_pose();

        if (current.frame_count != last_frame_count) {
            accumulated_yaw += normalize_angle(current.yaw_rad - previous.yaw_rad);
            previous = current;
            last_frame_count = current.frame_count;
            last_frame_tick = now;
            if (accumulated_yaw <= -NUEDC_2026_HALF_TURN_RAD) {
                return NUEDC_2026_STATUS_OK;
            }
        }

        if ((u32)(now - last_frame_tick) >= NUEDC_2026_ODOM_STALE_TIMEOUT_MS) {
            return NUEDC_2026_STATUS_ODOM_STALE;
        }
        if ((u32)(now - segment_start_tick) >= timeout_ms) {
            return NUEDC_2026_STATUS_SEGMENT_TIMEOUT;
        }
    }
}

static Nuedc2026Status run_capsule(const Nuedc2026RouteConfig *config)
{
    Nuedc2026Status status;

    status = run_straight(config->speed_mps, config->straight_timeout_ms);
    if (status != NUEDC_2026_STATUS_OK) return stop_with_status(status);

    status = run_clockwise_half_arc(
        config->radius_m,
        config->speed_mps,
        config->arc_omega_scale,
        config->arc_timeout_ms);
    if (status != NUEDC_2026_STATUS_OK) return stop_with_status(status);

    status = run_straight(config->speed_mps, config->straight_timeout_ms);
    if (status != NUEDC_2026_STATUS_OK) return stop_with_status(status);

    status = run_clockwise_half_arc(
        config->radius_m,
        config->speed_mps,
        config->arc_omega_scale,
        config->arc_timeout_ms);
    if (status != NUEDC_2026_STATUS_OK) return stop_with_status(status);

    return stop_with_status(NUEDC_2026_STATUS_OK);
}

Nuedc2026Status Nuedc2026_RunRoute(Nuedc2026RouteId route_id)
{
    Nuedc2026RouteConfig config;
    Nuedc2026Status status;

    if (route_id == NUEDC_2026_ROUTE_H) {
        config.radius_m = NUEDC_2026_H_RADIUS_M;
        config.speed_mps = NUEDC_2026_H_SPEED_MPS;
        config.arc_omega_scale = NUEDC_2026_H_ARC_OMEGA_SCALE;
        config.straight_timeout_ms = NUEDC_2026_H_STRAIGHT_TIMEOUT_MS;
        config.arc_timeout_ms = NUEDC_2026_H_ARC_TIMEOUT_MS;
    } else if (route_id == NUEDC_2026_ROUTE_D) {
        config.radius_m = NUEDC_2026_D_RADIUS_M;
        config.speed_mps = NUEDC_2026_D_SPEED_MPS;
        config.arc_omega_scale = NUEDC_2026_D_ARC_OMEGA_SCALE;
        config.straight_timeout_ms = NUEDC_2026_D_STRAIGHT_TIMEOUT_MS;
        config.arc_timeout_ms = NUEDC_2026_D_ARC_TIMEOUT_MS;
    } else {
        return stop_with_status(NUEDC_2026_STATUS_INVALID_ROUTE);
    }

    g_dfcom_unit_mode = DFCOM_UNIT_M;
    Cmd_Subscribe_Odom(ODOM_MODE_CONTINUOUS, ODOM_FREQ_100HZ);
    status = wait_for_live_odometry();
    if (status != NUEDC_2026_STATUS_OK) return stop_with_status(status);

    return run_capsule(&config);
}

Nuedc2026Status Nuedc2026_RunHRoute(void)
{
    return Nuedc2026_RunRoute(NUEDC_2026_ROUTE_H);
}

Nuedc2026Status Nuedc2026_RunDRoute(void)
{
    return Nuedc2026_RunRoute(NUEDC_2026_ROUTE_D);
}

Nuedc2026Status Nuedc2026_RunSelectedRoute(void)
{
    return Nuedc2026_RunRoute((Nuedc2026RouteId)NUEDC_2026_ACTIVE_ROUTE);
}

