//#include <time.h>
#include <stdio.h>      /*标准输入输出定义*/  
#include <stdlib.h>     /*标准函数库定义*/  
#include <unistd.h>     /*Unix 标准函数定义*/  
#include <sys/types.h>   
#include <sys/stat.h>     
#include <fcntl.h>      /*文件控制定义*/  
#include <termios.h>    /*PPSIX 终端控制定义*/  
#include <errno.h>      /*错误号定义*/  
#include <string.h>  
#include "stdbool.h"
#include <sys/time.h> 
#include <math.h>
#include "misc.h"
#include "config.h"
#include "uart.h"

extern void SetMotoSpeed(int fd_car, int left, int right);


int interval_ms;
s64 pg_start_timestamp;
s64 pg_end_timestamp;
static s64 ms_init;
int system_time_in_sec = 0;
int PID_Flag = 0;

//---- 获取系统当前时间戳(单位：ms) ----//
s64 GetCurrentTimeMs(void)
{
  s64 ms;
  struct timeval tv;
  struct timezone tz;
  gettimeofday(&tv, &tz);
  ms = (((s64)tv.tv_sec*(s64)1000) + ((s64)tv.tv_usec/(s64)1000));
  return ms;
}

void Get_MainLoopStartTime(void)
{
  pg_start_timestamp = GetCurrentTimeMs();
}

void* Get_MainLoopTotalTime(void)
{
  static s64 total_ms;
  pg_end_timestamp = GetCurrentTimeMs();
  total_ms = (pg_end_timestamp - pg_start_timestamp);
  //printf("\nTotal Test Time = %.3lf s\n", (double)(pg_end_timestamp - pg_start_timestamp) * 0.001d);
  return &total_ms;
}

void timer_init(void)
{
  ms_init = GetCurrentTimeMs();
}

void timer_check(void)
{
  static s64 n_100ms_bk = 0;
  s64 ms = GetCurrentTimeMs();

  system_time_in_sec = (ms - ms_init) / 1000;

  s64 n_100ms = (ms - ms_init) / 100;
  if(n_100ms_bk != n_100ms)
  {
    n_100ms_bk = n_100ms;
#if (SIMULATE_ENABLE) 
    PID_Flag = 1;
#endif
#if (0)
    PID_Flag = 1; // ****
#endif
  }
}


//---- 坐标系初始化 ----//
COORDINATE_PARAM* Coordinate_Init(double lati_m, double longti_m)
{
  COORDINATE_PARAM* coo = (COORDINATE_PARAM*) malloc(sizeof(COORDINATE_PARAM));
  if(coo != NULL)
  {
    coo->origin_latitude = lati_m;
    coo->origin_longitude = longti_m;
    coo->earth_r = 6371 * 1000.0;
    coo->current_lati_cycle_r = coo->earth_r * cos((lati_m / 60.0) * (PI / 180.0));
    coo->yaw_offset = 0; 
  }
  return coo;
}

void Coordinate_Release(COORDINATE_PARAM* param)
{
  if(param != NULL)
  {
    free(param);
  }
}

/* yaw  - 正北角度 [0,360)
   输出 - 笛卡尔坐标系弧度(-PI, PI]
   **** 待测试 ****
*/
double YAW2DIR(double yaw)
{
  if (yaw < 0.0d) yaw = 0.0d;
  else if (yaw >= 360.0d) yaw = 0.0d;
  double tmp = 90.0d - yaw; 
  if (tmp <= -180.0d)
    tmp = 360.0d + tmp;
  return tmp * (PI / 180.0d);
}

void Get_Coordinate(COORDINATE* co, double lati_m, double longti_m, double yaw, COORDINATE_PARAM* base)
{
  co->x = base->current_lati_cycle_r * ((1./60.)*(PI/180.)) * (longti_m - base->origin_longitude);
  co->y = base->earth_r * ((1./60.)*(PI/180.)) * (lati_m - base->origin_latitude);
  co->direction = yaw - base->yaw_offset; 
  co->direction = YAW2DIR(yaw - base->yaw_offset); //(-PI, PI]
}

LINE_SEGMENT_PARAM* Creat_LineSegment(double start_x, double start_y, double end_x, double end_y)
{
  LINE_SEGMENT_PARAM* line = (LINE_SEGMENT_PARAM*) malloc(sizeof(LINE_SEGMENT_PARAM));
  if(line != NULL)
  {
    line->start_x = start_x;
    line->start_y = start_y;
    line->k = (end_y - start_y) / (end_x - start_x);
    line->b = start_y - (line->k * start_x);
    line->end_x = end_x;
    line->end_y = end_y;
    line->length = sqrt((end_y - start_y) * (end_y - start_y) + (end_x - start_x) * (end_x - start_x));
  }
  return line;
}

void LineSegment_Release(LINE_SEGMENT_PARAM* param)
{
  if(param != NULL)
  {
    free(param);
  }
}
/*
**
*/
DEMO_01_PARAM* DEMO_01_Init(double lati_m_01, double longti_m_01, double lati_m_02, double longti_m_02)
{
  DEMO_01_PARAM* demo = (DEMO_01_PARAM*) malloc(sizeof(DEMO_01_PARAM));
  if(demo != NULL)
  {
    double default_diff_angle = 15.0;
    double default_run_distance = 5.0;
    double line_segment_angle;
    double vertical_line_angle;
    COORDINATE_PARAM* coo_base = Coordinate_Init(lati_m_01, longti_m_01);
    COORDINATE* point = (COORDINATE*) malloc(sizeof(COORDINATE));
    Get_Coordinate(point, lati_m_02, longti_m_02, 0.0, coo_base);

    line_segment_angle = atan2(point->y, point->x) * (180.0 / PI);
    vertical_line_angle = line_segment_angle - 90.0;
    LINE_SEGMENT_PARAM* line = Creat_LineSegment(0.0, 0.0, point->x, point->y);
    
    point->x = 0.0;
    point->y = 0.0;
    point->direction = 0.0;

    demo->coo = coo_base;
    demo->line = line;
    demo->current_location = point;
    demo->vertical_line_k = tan(vertical_line_angle * (PI / 180.0));
    demo->vertical_line_b = 0;

    demo->max_wheel_speed = MAX_WHEEL_SPEED_MMS * 0.001; 
    demo->left_speed = 0;
    demo->right_speed = 0;
    demo->acceleration = MAX_WHEEL_ACCELERATION_MMSS * 0.001; 
    demo->brake_distance = (demo->max_wheel_speed / demo->acceleration) * demo->max_wheel_speed * 0.5;
    demo->run_distance = 0;
    demo->expected_speed = demo->max_wheel_speed;
    demo->car_speed = 0;

    printf("\r\n---- DEMO_01 Infor ----\r\n");
    printf("line_segment_angle = %lf, vertical_line_angle = %lf \r\n", line_segment_angle, vertical_line_angle);
    printf("start_X = %lf, start_Y = %lf , desti_X = %lf, desti_Y = %lf , Length = %lf \r\n", \
      line->start_x, line->start_y, line->end_x, line->end_y, line->length);
    //printf("y = kx + b : k = %lf, b = %lf \r\n", line->k, line->b);
    //printf("brake_distance = %lf \r\n", demo->brake_distance);
    printf("---------------------------\r\n");
     	
    //---- PID系数 ----//
    demo->P = 1.0;
    demo->I = 0.00;
    demo->D = 0.00;
    demo->error = 0.0;
    demo->last_error = 0.0;
    demo->error_sum = 0.0;
    demo->max_error_sum =  10.0;
    demo->min_error_sum = -10.0;
    demo->max_error = 4.0; //[车体]到[既定路线]的距离最大为2米，大于这个值后立即停车
    demo->max_diff_rate = 0.2; //0.2

    //---- ----//
    demo->error_flag = 0;

  }
  return demo;
}

void DEMO_01_Release(DEMO_01_PARAM* param)
{
  if(param != NULL)
  {
    Coordinate_Release(param->coo);
    LineSegment_Release(param->line);
    if(param->current_location != NULL)
      free(param->current_location);
    free(param);
  }
}

void CAR_GoAhead_Task(DEMO_01_PARAM * param, int fd_car)
{
  double control_cycle = 0.1; //控制周期，100ms
  double l_coff = 1.0;
  double r_coff = l_coff - 0.30; // 0.1, 0.15, 0.2, 0.3 
  if(PID_Flag)
  {
    PID_Flag = 0;

    Car_RunLine(param, control_cycle, 63.0);

    if((param->left_speed > 0.0) && (param->right_speed > 0.0))
      printf("[l = %lf , r = %lf] \r\n", param->left_speed * l_coff, param->right_speed * r_coff);
    SetMotoSpeed(fd_car, param->left_speed * 1000.0d * l_coff , param->right_speed * 1000.0d * r_coff);
  }  
}

int DEMO_01_Task(DEMO_01_PARAM * param, GPSINFO* gps, int fd_car)
{
  double control_cycle = 0.1; //控制周期，100ms
  static int counter = 0;
  static int zero_speed_counter = 0;
  int ret = 0;
  if(PID_Flag)
  {
    PID_Flag = 0;

    Get_Coordinate(param->current_location, gps->latitude_InM, gps->longitude_InM, gps->Yaw, param->coo);
    //计算当前点到规定路径的垂线距离
    //y1 = k1*x1 + b1
    //y1 = k1*x1 + b2
    param->vertical_line_b = param->current_location->y - param->current_location->x * param->vertical_line_k;
    double k1,k2,b1,b2;
    k1 = param->line->k;
    b1 = param->line->b;
    k2 = param->vertical_line_k;
    b2 = param->vertical_line_b;
    
    double x1 = (b2 - b1) / (k1 - k2);
    double y1 = k1 * x1 + b1;
    double x0 = param->current_location->x;
    double y0 = param->current_location->y;
    double distance = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
    double diff = y0 - (k1 * x0 + b1);
    if(diff > 0.0) //点处于线的左上方
    {
      //右拐
      distance = -distance;
    }
    else 
    {
      //左拐
    }
    //---- 特殊情况处理 ----//
    if(fabs(distance) > param->max_error)
    {
      static int print = 1;
      if(print)
      {
        //print = 0;
        printf("Beyond max error! Error Code %d.\r\n", 0x0001);
      }
      param->left_speed = 0;
      param->right_speed = 0;
      param->error_flag = 1;
      SetMotoSpeed(fd_car, param->left_speed * 1000.0d, param->right_speed * 1000.0d);
      ret = -1;
      //goto SET_MOTO;
      return ret;
    }

    
    /* 设置终点减速逻辑 :
    ** (运行距离) > (线段长度-刹车距离)，开始减速
    */
    //if(param->run_distance > (param->line->length - param->brake_distance))
    if(param->run_distance > param->line->length)
    {
      param->expected_speed = 0;
    }
    // 车体前进方向加减速控制，实现匀加速和匀减速过程
    int fix_car_speed = param->car_speed * 10000.0;
    int fix_expected_speed = param->expected_speed * 10000.0;
    if(fix_car_speed < fix_expected_speed)
    {
      param->car_speed += param->acceleration * control_cycle;
      if(param->car_speed > param->expected_speed) param->car_speed = param->expected_speed;
      printf("speed ++ \r\n");
    }
    else if(fix_car_speed > fix_expected_speed) 
    {
      param->car_speed -= param->acceleration * control_cycle;
      if(param->car_speed < 0.0) param->car_speed = 0.0;
      printf("speed -- \r\n");
    }
    if(fix_car_speed < 100)  zero_speed_counter += 1;
    else zero_speed_counter = 0;
    if(zero_speed_counter > 30) 
    {
      param->left_speed = 0;
      param->right_speed = 0;
      ret = -1;
      printf("run_distance = %lf \r\n", param->run_distance);
      SetMotoSpeed(fd_car, param->left_speed * 1000.0d, param->right_speed * 1000.0d);
      //goto SET_MOTO;
      return ret;
    }
    
    // 运行距离积分
    param->run_distance += (param->car_speed * control_cycle);
    
    //----------------------//
    param->last_error = param->error;
    param->error = distance;
    param->error_sum += distance;
    if(param->error_sum > param->max_error_sum) param->error_sum = param->max_error_sum;
    else if(param->error_sum < param->min_error_sum) param->error_sum = param->min_error_sum;
    double pid_out = param->P * param->error + param->I * param->error_sum + param->D * (param->error - param->last_error);
    pid_out = MAX(pid_out, -1.0);
    pid_out = MIN(pid_out,  1.0);
    double diff_rate = pid_out * param->max_diff_rate;
    param->left_speed = param->car_speed * (1.0 - 0.5 * diff_rate);
    param->right_speed = param->car_speed * (1.0 + 0.5 * diff_rate);

    if((counter & 15) == 0)
    {
      printf("\r\n  curr_x = %lf | %lf, curr_y = %lf | %lf, [ error = %lf , pid_out = %lf] [l = %lf , r = %lf, %c] dis = %lf total = %lf\r\n", param->current_location->x, param->line->end_x, param->current_location->y,  param->line->end_y, param->error, pid_out, param->left_speed, param->right_speed, (param->left_speed > param->right_speed)?'R':'L', fabs(distance), param->run_distance);
    }

    if(counter < 10)
    {
      //printf("[l = %lf , r = %lf] \r\n", param->left_speed, param->right_speed);
    }
SET_MOTO:
    if(counter < 10)
      //printf("[l = %lf , r = %lf] \r\n", param->left_speed, param->right_speed);
    SetMotoSpeed(fd_car, param->left_speed * 1000.0d, param->right_speed * 1000.0d);
    counter += 1;
  }
  return ret;
}

//获取两大点间的距离
//输入两点待纬度和经度，单位：minute
double Get_TwoPointDistance(double lati_01, double longti_01, double lati_02, double longti_02)
{
    double earth_r = 6371 * 1000.0;
    double lati_cycle_r = earth_r * cos((lati_01+lati_02) * (PI / 180.0) * 0.5 / 60.0 );
    double dx = (longti_02 - longti_01) * (PI / 180.0) * (lati_cycle_r / 60.0);
    double dy = (lati_02 - lati_01) * (PI / 180.0) * (earth_r / 60.0);
    printf("\r\ndx = %lf, dy = %lf \r\n", dx, dy); //lati_r = %lf, 
    return sqrt(dx * dx + dy * dy);
}

void Car_RunLine(DEMO_01_PARAM * param, double cycle_time, double distance_meter)
{
  static int pro = 0;
  static int tick_1, tick_2, tick_3;
  switch(pro)
  {
  case 0:
    {
      double d1 = (param->max_wheel_speed * 0.5) * (param->max_wheel_speed / param->acceleration);
      double d2 = distance_meter - (d1 * 2.0);
      d2 = MAX(0.0, d2);
      double d123 = d1 + d2 + d1;
      tick_1 = (param->max_wheel_speed / param->acceleration) / cycle_time;
      tick_2 = (d2 / (param->max_wheel_speed) ) / cycle_time;
      tick_3 = tick_1;
      tick_1 = MAX(1, tick_1);
      tick_2 = MAX(1, tick_2);
      tick_3 = MAX(1, tick_3);
      printf("run_distance = %lf meter, T1 = %d, T2 = %d, T3 = %d \r\n", d123, tick_1, tick_2, tick_3);
      
      param->left_speed = 0;
      param->right_speed = 0;
      pro += 1;
    }
    break;
  case 1: 
    {
      if(tick_1 > 0)
      {
        param->left_speed += (param->acceleration * cycle_time);
        param->right_speed += (param->acceleration * cycle_time);
        tick_1 -= 1;
      }
      else
      {
        printf("T1 Finish, L = %lf , R = %lf \r\n", param->left_speed, param->right_speed);
        pro += 1;
      }
    }
    break;
  case 2: 
    {
      if(tick_2 > 0)
      {
        tick_2 -= 1;
      }
      else
      {
        printf("T2 Finish, L = %lf , R = %lf \r\n", param->left_speed, param->right_speed);
        pro += 1;
      }
    }
    break;
  case 3: 
    {
      if(tick_3 > 0)
      {
        param->left_speed -= (param->acceleration * cycle_time);
        param->right_speed -= (param->acceleration * cycle_time);
        tick_3 -= 1;
      }
      else
      {
        printf("T3 Finish, L = %lf , R = %lf \r\n", param->left_speed, param->right_speed);
        param->left_speed = 0;
        param->right_speed = 0;
        pro += 1;
      }
    }
    break;
  case 4:
    break;
  }
}

