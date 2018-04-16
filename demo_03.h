#ifndef _DEMO_03_H_
#define _DEMO_03_H_
#include "nmea.h"
#include "misc.h"
#include <stdio.h> 

typedef struct
{ //ky * y = kx * x + b
  double kx;
  double ky;
  double b;
  double start_x;
  double start_y;
  double end_x;
  double end_y;
  double length;
  double yaw;//弧度
  double speed_limit;

  double v_kx;
  double v_ky;
  double v_b;
}LINE_SEGMENT_PP_PARAM;


typedef struct
{
  COORDINATE_PARAM* coo;
  LINE_SEGMENT_PP_PARAM** line;
  COORDINATE* current_location;
  //double* v_kx;
  //double* v_ky;
  //double* v_b;
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
}DEMO_03_PARAM;


void DEMO_03_Release(DEMO_03_PARAM* param);
DEMO_03_PARAM* DEMO_03_Init(double lati_m[], double longti_m[], int num_point, FILE* log);
int DEMO_03_Task(DEMO_03_PARAM * param, GPSINFO* gps, int fd_car, FILE* log);
#endif



