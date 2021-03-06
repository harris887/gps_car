#include "misc.h"
#include "demo_02.h"
#include <stdio.h>      
#include <stdlib.h>    
#include <math.h>
#include "uart.h"

LINE_SEGMENT_PLUS_PARAM* Creat_LineSegmentPlus(double start_x, double start_y, double end_x, double end_y)
{
  LINE_SEGMENT_PLUS_PARAM* line = (LINE_SEGMENT_PLUS_PARAM*) malloc(sizeof(LINE_SEGMENT_PLUS_PARAM));
  if(line != NULL)
  {
    line->start_x = start_x;
    line->start_y = start_y;
    line->k = (end_y - start_y) / (end_x - start_x);
    line->b = start_y - (line->k * start_x);
    line->end_x = end_x;
    line->end_y = end_y;
    line->length = sqrt((end_y - start_y) * (end_y - start_y) + (end_x - start_x) * (end_x - start_x));
    line->yaw = atan2((end_y - start_y), (end_x - start_x));
  }
  return line;
}

void LineSegmentPlus_Release(LINE_SEGMENT_PLUS_PARAM* param)
{
  if(param != NULL)
  {
    free(param);
  }
}

/*
 * lati_m[]      - 纬度数组
 * longti_m_01[] - 经度数组
 * num_point     - 数组长度 (>=2)
*/
DEMO_02_PARAM* DEMO_02_Init(double lati_m[], double longti_m[], int num_point, FILE* log)
{
  if(num_point < 2) return NULL;
  int line_num = num_point - 1;
  DEMO_02_PARAM* demo = (DEMO_02_PARAM*) malloc(sizeof(DEMO_02_PARAM));
  if(demo != NULL)
  {
    //double default_diff_angle = 15.0;
    //double default_run_distance = 5.0;

    double line_start_x = 0.0, line_start_y = 0.0;
    COORDINATE_PARAM* coo_base = Coordinate_Init(lati_m[0], longti_m[0]);
    COORDINATE* point = (COORDINATE*) malloc(sizeof(COORDINATE));
    demo->vertical_line_k =(double*) malloc(sizeof(double) * line_num);
    demo->vertical_line_b =(double*) malloc(sizeof(double) * line_num);
    demo->line = (LINE_SEGMENT_PLUS_PARAM**) malloc(sizeof(LINE_SEGMENT_PLUS_PARAM*) * line_num);
    demo->line_num = line_num;
    // 生成 [num_point - 1] 条直线
    for(int i = 0; i < line_num; i++)
    {
      Get_Coordinate(point, lati_m[i + 1], longti_m[i + 1], 0.0, coo_base);
      LINE_SEGMENT_PLUS_PARAM* line = Creat_LineSegmentPlus(line_start_x, line_start_y, point->x, point->y);
      demo->vertical_line_k[i] = tan(line->yaw - PI * 0.5);
      demo->vertical_line_b[i] = 0.0d;
      demo->line[i] = line;
      line_start_x = point->x;
      line_start_y = point->y;
    }
    point->x = 0.0;
    point->y = 0.0;
    point->direction = 0.0;
    demo->coo = coo_base;
    demo->current_location = point;
    demo->cur_line_index = 0;

    //
    demo->max_wheel_speed = MAX_WHEEL_SPEED_MMS * 0.001; 
    demo->left_speed = 0;
    demo->right_speed = 0;
    demo->acceleration = MAX_WHEEL_ACCELERATION_MMSS * 0.001; 
    demo->brake_distance = (demo->max_wheel_speed / demo->acceleration) * demo->max_wheel_speed * 0.5;
    demo->run_distance = 0;
    demo->expected_speed = demo->max_wheel_speed;
    demo->car_speed = 0;

    printf("\r\n---- DEMO_02 Infor ----\r\n");
    //printf("line_segment_angle = %lf, vertical_line_angle = %lf \r\n", line_segment_angle, vertical_line_angle);
    //printf("start_X = %lf, start_Y = %lf , desti_X = %lf, desti_Y = %lf , Length = %lf \r\n", \
    //  line->start_x, line->start_y, line->end_x, line->end_y, line->length);
    //printf("y = kx + b : k = %lf, b = %lf \r\n", line->k, line->b);
    //printf("brake_distance = %lf \r\n", demo->brake_distance);
    printf("---------------------------\r\n");
     	
    //---- PID系数 ----//
    demo->yaw_P = 1.0;
    demo->P = 1.0;
    demo->I = 0.00;
    demo->D = 0.00;
    demo->error = 0.0;
    demo->last_error = 0.0;
    demo->error_sum = 0.0;
    demo->max_error_sum =  10.0;
    demo->min_error_sum = -10.0;
    demo->max_away_from_line = 4.0; //[车体]到[既定路线]的距离最大为4米，大于这个值后立即停车
    demo->max_diff_rate = 0.2; //0.2

    demo->min_distance_switch_line = 2.0; //1.0;

    //---- ----//
    demo->error_flag = 0;

    if(log != NULL)
    {
      fprintf(log, "cur_x \t cur_y \t cur_yaw \t line_x \t line_y \t line_yaw \t l_speed \t r_speed \n");
    }

  }
  return demo;
}

void DEMO_02_Release(DEMO_02_PARAM* param)
{
  if(param != NULL)
  {
    Coordinate_Release(param->coo);
    if(param->line != NULL)
    {
      for(int i = 0; i < param->line_num; i++)
      {
        LineSegmentPlus_Release(param->line[i]);
      }
      free(param->line);
    }
    if(param->current_location != NULL) free(param->current_location);
    if(param->vertical_line_k != NULL)  free(param->vertical_line_k);
    if(param->vertical_line_b != NULL)  free(param->vertical_line_b);
    free(param);
  }
}


int DEMO_02_Task(DEMO_02_PARAM * param, GPSINFO* gps, int fd_car, FILE* log)
{
  double control_cycle = 0.1; //控制周期，100ms
  static int counter = 0;
  static int zero_speed_counter = 0;
  int ret = 0;
  int cur_line_index;
  LINE_SEGMENT_PLUS_PARAM* cur_line;
  double yaw0;
  if(PID_Flag)
  {
    PID_Flag = 0;

    cur_line_index = param->cur_line_index;
    cur_line = param->line[cur_line_index];
    Get_Coordinate(param->current_location, gps->latitude_InM, gps->longitude_InM, gps->Yaw, param->coo);
    yaw0 = param->current_location->direction;
    //计算当前点到规定路径的垂线距离
    //y1 = k1*x1 + b1
    //y1 = k1*x1 + b2
    param->vertical_line_b[cur_line_index] = param->current_location->y - param->current_location->x * param->vertical_line_k[cur_line_index];
    double k1,k2,b1,b2;
    k1 = cur_line->k;
    b1 = cur_line->b;
    k2 = param->vertical_line_k[cur_line_index];
    b2 = param->vertical_line_b[cur_line_index];
    
    double x1 = (b2 - b1) / (k1 - k2);
    double y1 = k1 * x1 + b1;
    double x0 = param->current_location->x;
    double y0 = param->current_location->y;
    double distance = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
    double diff = y0 - (k1 * x0 + b1);
	  if ((yaw0 > -(PI * 0.5)) && (yaw0 < (PI * 0.5))) //(*重要改动*)
    {
      if(diff > 0.0) //点处于线的左上方
      {
        //右拐
        distance = -distance;
      }
    }
	  else
    {
      if(diff < 0.0) //点处于线的右下方
      {
        //右拐
        distance = -distance;
      }
    }	

    //---- 特殊情况处理 ----//
    if(fabs(distance) > param->max_away_from_line)
    {
      printf("Beyond max error! Error Code %d.\r\n", 0x0001);
      param->left_speed = 0;
      param->right_speed = 0;
      param->error_flag = 1;
      SetMotoSpeed(fd_car, param->left_speed * 1000.0d, param->right_speed * 1000.0d);
      ret = -1;
      return ret;
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
      printf("zero_speed last %lfs, run_distance = %lf \r\n", (double)zero_speed_counter * control_cycle, param->run_distance);
      SetMotoSpeed(fd_car, param->left_speed * 1000.0d, param->right_speed * 1000.0d);
      return ret;
    }
    
    // 运行距离积分
    param->run_distance += (param->car_speed * control_cycle);

    // 航向角处理，航向角差在目前的方案中限定在(-PI，PI] (*重要改动*)
    double diff_yaw = yaw0 - cur_line->yaw;
	  if (diff_yaw > PI)
    {
	    diff_yaw = diff_yaw - 2.0d * PI;
    }
	  else if  (diff_yaw <= -PI)
    {
	    diff_yaw = diff_yaw + 2.0d * PI;
    }
	  
    // 航向角差大于60度，距离不起效，这里可能处理不线性，后期要修改 (*重要改动*)
    if (fabs(diff_yaw) > (PI / 3.0))
    {
      distance = 0.0;
    }  

    
    //----------------------//
    param->last_error = param->error;
    param->error = distance;
    param->error_sum += distance;
    if(param->error_sum > param->max_error_sum) param->error_sum = param->max_error_sum;
    else if(param->error_sum < param->min_error_sum) param->error_sum = param->min_error_sum;
    double pid_out = param->P * param->error + param->I * param->error_sum + param->D * (param->error - param->last_error) + (-1.0) * diff_yaw * param->yaw_P;
    pid_out = MAX(pid_out, -1.0);
    pid_out = MIN(pid_out,  1.0);
    double diff_rate = pid_out * param->max_diff_rate;
    param->left_speed = param->car_speed * (1.0 - 0.5 * diff_rate);
    param->right_speed = param->car_speed * (1.0 + 0.5 * diff_rate);

    if((counter & 15) == 0)
    {
      printf("\r\n  curr_x = %lf | %lf, curr_y = %lf | %lf, curr_yaw = = %lf | %lf [ error = %lf , pid_out = %lf] [l = %lf , r = %lf, %c] dis = %lf total = %lf\r\n", \
        param->current_location->x, cur_line->end_x, param->current_location->y,  cur_line->end_y, yaw0, cur_line->yaw, \
        param->error, pid_out, param->left_speed, param->right_speed, \
        (param->left_speed > param->right_speed)?'R':'L', fabs(distance), param->run_distance);
    }

    SetMotoSpeed(fd_car, param->left_speed * 1000.0d, param->right_speed * 1000.0d);

    if(log != NULL)
    {
      //fprintf(log, "cur_x \t cur_y \t cur_yaw \t line_x \t line_y \t line_yaw \t l_speed \t r_speed \n");
      fprintf(log, "%lf \t %lf \t %lf \t %lf \t %lf \t %lf \t %lf \t %lf \n", \
        param->current_location->x, param->current_location->y, param->current_location->direction, \
        x1, y1, cur_line->yaw, \
        param->left_speed, param->right_speed);
    }

	  //---- 线段切换 ----//
	  double dis_end =  sqrt((cur_line->end_x - x1) * (cur_line->end_x - x1) + (cur_line->end_y - y1) * (cur_line->end_y - y1));
	  if (dis_end <  param->min_distance_switch_line)
    {
	    if (cur_line_index < (param->line_num - 1))
      {
        cur_line_index = cur_line_index + 1;
        param->cur_line_index = cur_line_index;
        printf("---- Change Line ----\r\n");
      }
	    else
      {//快到终点[停车]
	      param->expected_speed = 0;
	    }
    }
    counter += 1;
  }
  return ret;
}



