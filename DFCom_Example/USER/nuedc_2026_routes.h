#ifndef NUEDC_2026_ROUTES_H
#define NUEDC_2026_ROUTES_H

/* -------------------------------------------------------------------------
 * 2026 电赛 H/D 题固定胶囊赛道：唯一调参入口
 *
 * 坐标与单位全部使用 DFCom 原生 SI：m、m/s、rad、rad/s。
 * 路线只读取里程计，不读取光电、灰度或其他循线传感器。
 * ------------------------------------------------------------------------- */

typedef enum {
    NUEDC_2026_ROUTE_H = 1,
    NUEDC_2026_ROUTE_D = 2
} Nuedc2026RouteId;

typedef enum {
    NUEDC_2026_STATUS_OK = 0,
    NUEDC_2026_STATUS_INVALID_ROUTE = 1,
    NUEDC_2026_STATUS_ODOM_STALE = 2,
    NUEDC_2026_STATUS_SEGMENT_TIMEOUT = 3
} Nuedc2026Status;

/* 改这一行即可选择烧录后运行 H 题或 D 题。 */
#define NUEDC_2026_ACTIVE_ROUTE              NUEDC_2026_ROUTE_H

/* 上电握手完成后留给摆车和人员撤离的时间；按 Reset 可重新开始一圈。 */
#define NUEDC_2026_START_DELAY_MS             3000U

/* 公共几何与控制节拍。 */
#define NUEDC_2026_STRAIGHT_LENGTH_M          1.500f
#define NUEDC_2026_HALF_TURN_RAD              3.141592654f
#define NUEDC_2026_CONTROL_PERIOD_MS          10U
#define NUEDC_2026_ODOM_STALE_TIMEOUT_MS      300U
#define NUEDC_2026_ODOM_START_TIMEOUT_MS      1000U

/* H 题：50 cm 半径；0.34 m/s 的理想整圈时间约 18.1 s。 */
#define NUEDC_2026_H_RADIUS_M                 0.500f
#define NUEDC_2026_H_SPEED_MPS                0.340f
#define NUEDC_2026_H_ARC_OMEGA_SCALE          1.000f
#define NUEDC_2026_H_STRAIGHT_TIMEOUT_MS      6500U
#define NUEDC_2026_H_ARC_TIMEOUT_MS           6500U

/* D 题：与已确认印刷地图一致为 741 mm；官方 750 mm 场地改成 0.750f。 */
#define NUEDC_2026_D_RADIUS_M                 0.741f
#define NUEDC_2026_D_SPEED_MPS                0.250f
#define NUEDC_2026_D_ARC_OMEGA_SCALE          1.000f
#define NUEDC_2026_D_STRAIGHT_TIMEOUT_MS      9000U
#define NUEDC_2026_D_ARC_TIMEOUT_MS           12000U

Nuedc2026Status Nuedc2026_RunRoute(Nuedc2026RouteId route_id);
Nuedc2026Status Nuedc2026_RunHRoute(void);
Nuedc2026Status Nuedc2026_RunDRoute(void);
Nuedc2026Status Nuedc2026_RunSelectedRoute(void);

#endif
