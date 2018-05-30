#ifndef _MISC_H_
#define _MISC_H_
#include "nmea.h"
#include "type_def.h"


#define MAX_WHEEL_SPEED_MMS          500.0 //800.0 500.0 //300.0, 400.0, [500.0], 600.0, 700.0     //轮子最大速度，单位：mm/s
#define MAX_WHEEL_ACCELERATION_MMSS  125.0     //轮子最大加速度，单位：mm/s^2       

//---------------------//
#define STRING_NUM_DECIMAL  "0123456789"
#define STRING_NUM_HEX      "0123456789ABCDEFabcdef" 
#ifndef PI
#define PI 3.1415926535898
#endif 

#define MAX(a, b)   (a > b) ? a : b
#define MIN(a, b)   (a < b) ? a : b

//---------------------//
extern int system_time_in_sec;



s64 GetCurrentTimeMs(void);
void Get_MainLoopStartTime(void);
void* Get_MainLoopTotalTime(void);
void timer_init(void);
void timer_check(void);

//---------------------//
typedef struct
{
  double origin_latitude;  //unit: minute
  double origin_longitude; //unit: minute
  double earth_r;          //unit: meter
  double current_lati_cycle_r; //unit: meter
  double yaw_offset; //unit: degree
}COORDINATE_PARAM;

typedef struct
{
  double x;
  double y;
  double direction;
}COORDINATE;

typedef struct
{ //y = k*x + b
  double k;
  double b;
  double start_x;
  double start_y;
  double end_x;
  double end_y;
  double length;
}LINE_SEGMENT_PARAM;

typedef struct
{
  COORDINATE_PARAM* coo;
  LINE_SEGMENT_PARAM* line;
  COORDINATE* current_location;
  double vertical_line_k;
  double vertical_line_b;
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
  double P;              // 误差系数
  double I;              // 误差积分系数
  double D;              // 误差微分系数
  double error;          // 误差值
  double last_error;     // 误差微分值
  double error_sum;      // 误差积分值
  double max_error_sum;  // 误差积分最大限定值，单位：meter。
  double min_error_sum;  // 误差积分最小限定值，单位：米。
  double max_error;      // 偏离预订轨道的最大距离，单位：米。（超过该值后紧急停车）
  double max_diff_rate;  // 两侧轮子速度差/速度的最大比例，(Va-Vb)/max(Va-Vb) = [0,max_diff_rate]

  //---
  int error_flag;
}DEMO_01_PARAM;

extern int PID_Flag;

COORDINATE_PARAM* Coordinate_Init(double lati_m, double longti_m);
void Coordinate_Release(COORDINATE_PARAM* param);
void Get_Coordinate(COORDINATE* co, double lati_m, double longti_m, double yaw, COORDINATE_PARAM* base);
//LINE_SEGMENT_PARAM* Creat_LineSegment(double start_x, double start_y, double direction_degree, double length);
//DEMO_01_PARAM* DEMO_01_Init(double lati_m, double longti_m, double yaw);
LINE_SEGMENT_PARAM* Creat_LineSegment(double start_x, double start_y, double end_x, double end_y);
DEMO_01_PARAM* DEMO_01_Init(double lati_m_01, double longti_m_01, double lati_m_02, double longti_m_02);
int DEMO_01_Task(DEMO_01_PARAM * param, GPSINFO* gps, int fd_car);
void DEMO_01_Release(DEMO_01_PARAM* param);
double Get_TwoPointDistance(double lati_01, double longti_01, double lati_02, double longti_02);
void Car_RunLine(DEMO_01_PARAM * param, double cycle_time, double distance_meter);
void CAR_GoAhead_Task(DEMO_01_PARAM * param, int fd_car);
double YAW2DIR(double yaw);
#endif
