#include "module_core.h"
#include <stdio.h>      
#include <stdlib.h>    
#include <math.h>
#include <string.h>
#include "modbus.h"
#include "misc.h"
#include "demo_03.h"
#include "uart.h"
#include "config.h"

//MODULE_CORE_PARAM* MODULE_CORE_Param = NULL;
int init_COORDINATE_flag = 1;
int init_ROAD_MAP_flag = 1;

MODULE_CORE_PARAM* MODULE_CORE_Init(FILE* log)
{
  MODULE_CORE_PARAM* param = (MODULE_CORE_PARAM*) malloc(sizeof(MODULE_CORE_PARAM));
  if(param != NULL)
  {
    memset(param, 0, sizeof(MODULE_CORE_PARAM));

    double speed_limit = 0.5;
    /*
    double line_start_x = 0.0, line_start_y = 0.0;
    COORDINATE_PARAM* coo_base = Coordinate_Init(lati_m[0], longti_m[0]);
    COORDINATE* point = (COORDINATE*) malloc(sizeof(COORDINATE));
    demo->line = (LINE_SEGMENT_PP_PARAM**) malloc(sizeof(LINE_SEGMENT_PP_PARAM*) * line_num);
    demo->line_num = line_num;
    // 生成 [num_point - 1] 条直线
    for(int i = 0; i < line_num; i++)
    {
      Get_Coordinate(point, lati_m[i + 1], longti_m[i + 1], 0.0, coo_base);
      LINE_SEGMENT_PP_PARAM* line = Creat_LineSegmentPP(line_start_x, line_start_y, point->x, point->y, speed_limit);
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
    */

    //
    param->max_wheel_speed = MAX_WHEEL_SPEED_MMS * 0.001; 
    param->left_speed = 0;
    param->right_speed = 0;
    param->acceleration = MAX_WHEEL_ACCELERATION_MMSS * 0.001; 
    param->brake_distance = (param->max_wheel_speed / param->acceleration) * param->max_wheel_speed * 0.5;
    param->run_distance = 0;
    param->expected_speed = param->max_wheel_speed;
    param->car_speed = 0;

    /*
    printf("\r\n---- module core Infor ----\r\n");
    //printf("line_segment_angle = %lf, vertical_line_angle = %lf \r\n", line_segment_angle, vertical_line_angle);
    //printf("start_X = %lf, start_Y = %lf , desti_X = %lf, desti_Y = %lf , Length = %lf \r\n", \
    //  line->start_x, line->start_y, line->end_x, line->end_y, line->length);
    //printf("y = kx + b : k = %lf, b = %lf \r\n", line->k, line->b);
    //printf("brake_distance = %lf \r\n", demo->brake_distance);
    printf("---------------------------\r\n");
    */
     	
    //---- PID系数 ----//
    param->yaw_P = 1.0;
    param->P = 1.0;
    param->I = 0.00;
    param->D = 0.00;
    param->error = 0.0;
    param->last_error = 0.0;
    param->error_sum = 0.0;
    param->max_error_sum =  10.0;
    param->min_error_sum = -10.0;
    param->max_away_from_line = 4.0; //4.0; //[车体]到[既定路线]的距离最大为4米，大于这个值后立即停车
    param->max_diff_rate = 0.2; //0.2
    param->min_distance_switch_line = 2.0; //1.0;

    //---- ----//
    param->error_flag = 0;

    if(log != NULL)
    {
      fprintf(log, "cur_x \t cur_y \t cur_yaw \t line_x \t line_y \t line_yaw \t l_speed \t r_speed \n");
    }

  }
  return param;
}

void MODULE_CORE_Release(MODULE_CORE_PARAM* param)
{
  if(param != NULL)
  {
    Coordinate_Release(param->coo);
    param->coo = NULL;

    // .....

    free(param);
  }
}

void MODULE_CORE_Task(MODULE_CORE_PARAM* param, GPSINFO* gps, int fd_car, FILE* log)
{
  static int pro = 0;
  if((init_COORDINATE_flag) && (pro ==0 ))
  {
    init_COORDINATE_flag = 0;
    if(param != NULL)
    {
      u64 longti = (((u64)MOD_BUS_Reg.ORIGIN_LONGTI[0] << 48) | ((u64)MOD_BUS_Reg.ORIGIN_LONGTI[1] << 32) | ((u64)MOD_BUS_Reg.ORIGIN_LONGTI[2] << 16) | ((u64)MOD_BUS_Reg.ORIGIN_LONGTI[3] << 0));
      u64 lati = (((u64)MOD_BUS_Reg.ORIGIN_LATI[0] << 48) | ((u64)MOD_BUS_Reg.ORIGIN_LATI[1] << 32) | ((u64)MOD_BUS_Reg.ORIGIN_LATI[2] << 16) | ((u64)MOD_BUS_Reg.ORIGIN_LATI[3] << 0)); 

      double longti_m, lati_m;
      Coordinate_Release(param->coo);
      param->coo = NULL;
      longti_m = *(double*)&longti * 60.0d;
      lati_m = *(double*)&lati * 60.0d;

      printf("\r\norgin_lati = %f, orgin_longti = %f\r\n", *(double*)&lati, *(double*)&longti);
      //printf("orgin_lati_m = %f, orgin_longti_m = %f\r\n", lati_m, longti_m);
      param->coo = Coordinate_Init(lati_m, longti_m); 
      printf("earth_r = %f km, current_lati_cycle_r = %f km\r\n", param->coo->earth_r * 0.001d, param->coo->current_lati_cycle_r * 0.001d);
    }
  }
  if((MOD_BUS_Reg.VEHICLE_CONTROL != 0) && (pro ==0 ))
  {
    if ((MOD_BUS_Reg.MAP_NUM > 0) && (MOD_BUS_Reg.MAP_NUM <= 16)&& (MOD_BUS_Reg.MAP_INDEX <= MOD_BUS_Reg.MAP_NUM) && (MOD_BUS_Reg.MAP_INDEX > 0))
    {
      int map_index = MOD_BUS_Reg.MAP_INDEX - 1;
      int map_start_reg_index, map_reg_num;
      map_start_reg_index = MOD_BUS_Reg.MAP_INFOR[map_index][0];
      map_reg_num = MOD_BUS_Reg.MAP_INFOR[map_index][1];
      if((map_reg_num >= 6) && (map_start_reg_index >= 0x400) && ((map_start_reg_index & 1) == 0))
      {
        MAP_PARAM* map = (MAP_PARAM*) (MOD_BUS_Reg.MAP_DATA + (map_start_reg_index - 0x400));
        if(map->Map_Type == 0)
        {
          if((map->Point_Num >= 2) && (map->Point_Num <= 128))
          {
            COORDINATE_PARAM* coo_base = param->coo;

            if(param->coo != NULL)
            {
              // release first
              if(param->line != NULL)
              {
                int i;
                for(i = 0; i < param->line_num; i++)
                {
                  LineSegmentPP_Release(param->line[i]);
                  param->line[i] = NULL;
                }
                free(param->line);
                param->line = NULL;
              }
              if(param->current_location != NULL) 
              {
                free(param->current_location);
                param->current_location = NULL;
              }
              // rebuild lines
              if(1)
              {
                unsigned int temp;
                int x_cm, y_cm;
                double speed_limit = 0.5;
                COORDINATE* point = (COORDINATE*) malloc(sizeof(COORDINATE));
                int line_num = map->Point_Num - 1, i;
                
                temp = (((unsigned int)map->Point_Coordinate_XY[0][0] << 16) | (unsigned int)map->Point_Coordinate_XY[0][1]);
                x_cm = *(int*)&temp;
                temp = (((unsigned int)map->Point_Coordinate_XY[0][2] << 16) | (unsigned int)map->Point_Coordinate_XY[0][3]);
                y_cm = *(int*)&temp;
                double line_start_x = x_cm * 0.01, line_start_y = y_cm * 0.01;
                param->line = (LINE_SEGMENT_PP_PARAM**) malloc(sizeof(LINE_SEGMENT_PP_PARAM*) * line_num);
                param->line_num = line_num;
                printf("\r\n\r\n---------------- create %03d lines ----------------\r\n", line_num);
                for(i = 0; i < line_num; i++)
                {
                  double x,y;
                  temp = (((unsigned int)map->Point_Coordinate_XY[i+1][0] << 16) | (unsigned int)map->Point_Coordinate_XY[i+1][1]);
                  x_cm = *(int*)&temp;
                  temp = (((unsigned int)map->Point_Coordinate_XY[i+1][2] << 16) | (unsigned int)map->Point_Coordinate_XY[i+1][3]);
                  y_cm = *(int*)&temp;
                  x = x_cm * 0.01;
                  y = y_cm * 0.01;
                  LINE_SEGMENT_PP_PARAM* line = (LINE_SEGMENT_PP_PARAM*) Creat_LineSegmentPP(line_start_x, line_start_y, x, y, speed_limit);
                  param->line[i] = line;
                  line_start_x = x;
                  line_start_y = y;
                  printf("Function: %.2f*y = %.2f*x + %.2f , Length: %.2f meter .\r\n", param->line[i]->ky, param->line[i]->kx, param->line[i]->b, param->line[i]->length);
                }
                printf("--------------------------------------------------\r\n\r\n");
                point->x = 0.0;
                point->y = 0.0;
                point->direction = 0.0;
                param->current_location = point;
                param->cur_line_index = 0;
                pro = 1;
                
              }
            }
            else
            {
              printf("error code 10004, COORDINATE_ORIGIN not init\r\n");
            }
          }
          else
          {
            printf("error code 10003, Point_Num = %d, support value [2,%d]\r\n", map->Point_Num, MAX_POINT_NUM_IN_ONE_MAP);
          }
        }
        else
        {
          printf("error code 10002, Map_Type = %d, support value [0, 0]\r\n", map->Map_Type);
        }
      }
      else
      {
        printf("error code 10001, map_index = %d, map_start_reg_index = 0x%x, map_reg_num = %d\r\n", map_index, map_start_reg_index, map_reg_num);
      }
    }
    else
    {
      printf("error code 10000, MAP_NUM = %d, MAP_INDEX = %d\r\n", MOD_BUS_Reg.MAP_NUM, MOD_BUS_Reg.MAP_INDEX);
    }
  }

  if(pro == 1)
  {
    int ret = VEHICLE_Run(param, gps, fd_car, log);
    if (ret < 0) 
    {

      MOD_BUS_Reg.VEHICLE_CONTROL = 0; // force stop
      VEHICLE_Reset(param);
      printf("VEHICLE_Run Finish !\r\n");
      pro = 0;
    }

    if(MOD_BUS_Reg.VEHICLE_CONTROL == 0)
    {
      VEHICLE_Reset(param);
      SetMotoSpeed(fd_car, 0, 0);
      printf("VEHICLE Force Stop !\r\n");
      pro = 0;
    }
  }

  if(pro == 0)
  {
    static u16 test_control_bk = 0;
    if(PID_Flag)
    {
      COORDINATE current_location; 
      short speed = 0;  
#if (0) //SIMULATE_ENABLE ****
      short speed_list[8] = {-100, 30, -20, -200, 18, 128 ,-35 , 188};
      static int speed_list_index = 0;
      speed = speed_list[speed_list_index];
      speed_list_index = (speed_list_index + 1) & 7;
      //printf("speed = %d\r\n", speed);
#endif
      PID_Flag = 0;

      Get_Coordinate(&current_location, gps->latitude_InM, gps->longitude_InM, gps->Yaw, param->coo);
      UpdateModBusRegs((int) (current_location.x * 100.0d), (int) (current_location.y * 100.0d), (short) current_location.direction , speed);
    }
    if(MOD_BUS_Reg.VEHICLE_TEST_CONTROL != test_control_bk)
    {
      test_control_bk = MOD_BUS_Reg.VEHICLE_TEST_CONTROL;
      if(test_control_bk) 
      {
        SetMotoSpeed(fd_car, 100, 100); //mm/s
        printf("\r\ntest vehicle forward , 100 mm/s fixed \r\n");
      }
      else
      {
        SetMotoSpeed(fd_car, 0, 0);   
        printf("\r\ntest vehicle stop ! \r\n");
      }
    }
  }
}

void UpdateModBusRegs(int x, int y, short yaw , short speed)
{
  unsigned int car_location_x = *(unsigned int*)&x;
  unsigned int car_location_y = *(unsigned int*)&y;
  short car_current_yaw = yaw;
  short car_speed_unit_10cms = speed;
#if (1) //****
  double gps_longti = gps.longitude;
  double gps_lati = gps.latitude;
  float gps_yaw = gps.Yaw;
  unsigned short location_quality = gps.FixMode;
  unsigned short yaw_quality = gps.AVR_FixMode;
#else
  double gps_longti = 12.34;
  double gps_lati = 67.89;
  float gps_yaw = 11.88;
  unsigned short location_quality = 4;
  unsigned short yaw_quality = 3;
#endif

  u16* longti = (u16*) &gps_longti;
  u16* lati = (u16*) &gps_lati;
  u16* g_yaw = (u16*) &gps_yaw;
  //printf("x %d, y %d\r\n", x, y);
  MOD_BUS_Reg.VEHICLE_LOCATION_X[0] = car_location_x >> 16;
  MOD_BUS_Reg.VEHICLE_LOCATION_X[1] = car_location_x & 0xFFFF;
  MOD_BUS_Reg.VEHICLE_LOCATION_Y[0] = car_location_y >> 16;
  MOD_BUS_Reg.VEHICLE_LOCATION_Y[1] = car_location_y & 0xFFFF;
  MOD_BUS_Reg.VEHICLE_LOCATION_YAW = car_current_yaw;
  MOD_BUS_Reg.VEHICLE_SPEED = car_speed_unit_10cms;
  MOD_BUS_Reg.VEHICLE_LONGTI[3] = longti[0];
  MOD_BUS_Reg.VEHICLE_LONGTI[2] = longti[1];
  MOD_BUS_Reg.VEHICLE_LONGTI[1] = longti[2];
  MOD_BUS_Reg.VEHICLE_LONGTI[0] = longti[3];
  MOD_BUS_Reg.VEHICLE_LATI[3] = lati[0];
  MOD_BUS_Reg.VEHICLE_LATI[2] = lati[1];
  MOD_BUS_Reg.VEHICLE_LATI[1] = lati[2];
  MOD_BUS_Reg.VEHICLE_LATI[0] = lati[3];
  MOD_BUS_Reg.VEHICLE_YAW[1] = g_yaw[0];
  MOD_BUS_Reg.VEHICLE_YAW[0] = g_yaw[1];
  MOD_BUS_Reg.GPS_LOCATION_QUALITY = location_quality;
  MOD_BUS_Reg.GPS_YAW_QUALITY = yaw_quality;
}

int VEHICLE_Reset(MODULE_CORE_PARAM * param)
{
    param->max_wheel_speed = MAX_WHEEL_SPEED_MMS * 0.001; 
    param->left_speed = 0;
    param->right_speed = 0;
    param->acceleration = MAX_WHEEL_ACCELERATION_MMSS * 0.001; 
    param->brake_distance = (param->max_wheel_speed / param->acceleration) * param->max_wheel_speed * 0.5;
    param->run_distance = 0;
    param->expected_speed = param->max_wheel_speed;
    param->car_speed = 0;

     	
    //---- PID系数 ----//
    param->yaw_P = 1.0;
    param->P = 1.0;
    param->I = 0.00;
    param->D = 0.00;
    param->error = 0.0;
    param->last_error = 0.0;
    param->error_sum = 0.0;
    param->max_error_sum =  10.0;
    param->min_error_sum = -10.0;
    param->max_away_from_line = 4.0; //[车体]到[既定路线]的距离最大为4米，大于这个值后立即停车
    param->max_diff_rate = 0.2; //0.2
    param->min_distance_switch_line = 2.0; //1.0;

    //---- ----//
    param->error_flag = 0;
}

int VEHICLE_Run(MODULE_CORE_PARAM * param, GPSINFO* gps, int fd_car, FILE* log)
{
  double control_cycle = 0.1; //控制周期，100ms
  static int counter = 0;
  static int zero_speed_counter = 0;
  int ret = 0;
  int cur_line_index;
  LINE_SEGMENT_PP_PARAM* cur_line;
  double yaw0;
  if(PID_Flag)
  {
    PID_Flag = 0;

    cur_line_index = param->cur_line_index;
    cur_line = param->line[cur_line_index];
    Get_Coordinate(param->current_location, gps->latitude_InM, gps->longitude_InM, gps->Yaw, param->coo);
    yaw0 = param->current_location->direction;
    //计算当前点到规定路径的垂线距离
    cur_line->v_b = cur_line->v_ky * param->current_location->y - cur_line->v_kx * param->current_location->x;

    double kx1,ky1,kx2,ky2,b1,b2;
    kx1 = cur_line->kx;
    ky1 = cur_line->ky;
    b1 = cur_line->b;
    kx2 = cur_line->v_kx;
    ky2 = cur_line->v_ky;
    b2 = cur_line->v_b;

    double x1,y1;
    if (ky1 == 0.0)
    {
      x1 = -b1 / kx1;
      y1 = b2 / ky2;
    }
    else if (kx1 == 0.0)
    {
      y1 = b1 / ky1;
      x1 = -b2 / kx2;
    }
    else
    {
      x1 = ((b2 / ky2) - (b1 / ky1)) / ((kx1 / ky1) - (kx2 / ky2));
      y1 = (kx1 * x1 + b1) / ky1;
    }
    
    double x0 = param->current_location->x;
    double y0 = param->current_location->y;
    double distance = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
    double diff = ky1 * y0 - (kx1 * x0 + b1);

    if (ky1 == 0.0)
    {
      if((diff > 0.0) && (yaw0 > 0.0))
      {
        distance = -distance;
      }
      else if ((diff < 0.0) && (yaw0 < 0.0))
      {
        distance = -distance;
      }
    }
    else
    {
      if ((yaw0 > -(PI * 0.5)) && (yaw0 < (PI * 0.5))) //(*重要改动*)
      {  
        if(diff > 0.0) //点处于线的左上方
        {  //右拐
          distance = -distance;
        }
      }
      else
      {
        if(diff < 0.0) //点处于线的右下方
        {  //右拐
          distance = -distance;
        }
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

    UpdateModBusRegs((int) (param->current_location->x * 100.0d), (int) (param->current_location->y * 100.0d), (short) yaw0 , (short)(param->car_speed * 10.0));
  }
  return ret;
}

void Show_MapInfor(void)
{
  int i, j;
  printf("\r\n");
  printf("=============================================\r\n");
  printf("MAP_NUM    = %d \r\n", MOD_BUS_Reg.MAP_NUM);
  printf("MAP_SELECT = %d \r\n", MOD_BUS_Reg.MAP_INDEX);
  printf("=============================================\r\n");
  if(MOD_BUS_Reg.MAP_NUM == 0) return;

  for(i = 0; i < MOD_BUS_Reg.MAP_NUM; i++)
  {
    int start_reg = MOD_BUS_Reg.MAP_INFOR[i][0];
    int map_reg_num = MOD_BUS_Reg.MAP_INFOR[i][1];
    printf("--------------------------------- MAP_%03d ---\r\n", i + 1);
    printf("  MAP_START_REG = %d\r\n", start_reg - 0x400);
    printf("  MAP_REG_NUM   = %d\r\n", map_reg_num);
    MAP_PARAM* map = (MAP_PARAM*) (MOD_BUS_Reg.MAP_DATA + (start_reg - 0x400));
    if(map->Map_Type == 0)
    {
      if((2 + map->Point_Num * 4) == map_reg_num)
      {
        printf("  POINT_NUM     = %d\r\n", map->Point_Num);
        for(j = 0; j < map->Point_Num; j++)
        {
          int x_cm, y_cm ;
          unsigned int temp;
          printf("---------------------------------------------\r\n");
          temp = (((unsigned int)map->Point_Coordinate_XY[j][0] << 16) | (unsigned int)map->Point_Coordinate_XY[j][1]);
          x_cm = *(int*)&temp;
          temp = (((unsigned int)map->Point_Coordinate_XY[j][2] << 16) | (unsigned int)map->Point_Coordinate_XY[j][3]);
          y_cm = *(int*)&temp;
          printf("    [% .2f, % .2f] \r\n", x_cm * 0.01, y_cm * 0.01);
        }
      }
      else
      {
        printf("  Error: Wrong Map_Data_Size !\r\n");
      }
    }
    else
    {
      printf("  Error: Unsupport Map_Type !\r\n");
    }
    printf("=============================================\r\n");
  }
}
