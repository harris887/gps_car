#ifndef _module_core_h_
#define _module_core_h_
#include "misc.h"
#include "demo_03.h"


typedef struct
{
  COORDINATE_PARAM* coo;
  LINE_SEGMENT_PP_PARAM** line;
  COORDINATE* current_location;

  int line_num;
  int cur_line_index;
  //---------------//
  double max_wheel_speed; // unit:mm/s
  double left_speed;      // unit:mm/s
  double right_speed;     // unit:mm/s
  double acceleration;    // unit:mm/s^2
  double brake_distance;  // 刹车距离
  double run_distance; //
  double expected_speed;
  double car_speed;

  //---- PID ----//
  double yaw_P;
  double P;              // 误差系数
  double I;              // 误差积分系数
  double D;              // 误差微分系数
  double error;          // 误差值
  double last_error;     // 误差微分值
  double error_sum;      // 误差积分值
  double max_error_sum;  // 误差积分最大限定值，单位：meter。
  double min_error_sum;  // 误差积分最小限定值，单位：米。
  double max_diff_rate;  // 两侧轮子速度差/速度的最大比例，(Va-Vb)/max(Va-Vb) = [0,max_diff_rate]
  
  //---- limit ----//
  double max_away_from_line;      // 偏离预订轨道的最大距离，单位：米。（超过该值后紧急停车）
  double min_distance_switch_line;

  //---
  int error_flag;
}MODULE_CORE_PARAM;

#define MAX_POINT_NUM_IN_ONE_MAP  128
typedef struct
{
  u16 Map_Type;            // 地图数据类型：0-点图，其他：保留
  u16 Point_Num;           // 地图中点的个数，数值>=2
  u16 Point_Coordinate_XY[MAX_POINT_NUM_IN_ONE_MAP][4];  // 点1的X坐标，单位：cm
}MAP_PARAM;


//extern MODULE_CORE_PARAM* MODULE_CORE_Param;
extern int init_COORDINATE_flag;

extern MODULE_CORE_PARAM* MODULE_CORE_Init(FILE* log);
extern void MODULE_CORE_Release(MODULE_CORE_PARAM* param);
extern void MODULE_CORE_Task(MODULE_CORE_PARAM* param, GPSINFO* gps, int fd_car, FILE* log);

extern void UpdateModBusRegs(int x, int y, short yaw , short speed);

extern int VEHICLE_Run(MODULE_CORE_PARAM * param, GPSINFO* gps, int fd_car, FILE* log);
extern void Show_MapInfor(void);
extern int VEHICLE_Reset(MODULE_CORE_PARAM * param);

#endif

