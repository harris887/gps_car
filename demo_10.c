#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "string.h"
#include "time.h"
#include "assert.h"
#include "demo_10.h"
#include "nmea.h"
#include "module_core.h"
#include "vehicle.h"

#define pi               3.1415926
#define Coff_Degree2Rad  (pi / 180.0)
#define Coff_Rad2Degree  (180.0 / pi)
#define max(x,y)         (x > y ? x : y)
#define min(x,y)         (x < y ? x : y)




void demo_10_simulate(void)
{
  printf("demo_10_simulate\n");
  /*
  point_list* aa;
  point a = {0, 0};
  aa = get_arc_point(&a, 5.0, 0.5 * 3.1415926, 0.0);
  if(aa != NULL)
  {
    printf("point_num = %d \n", aa->point_num);
  }
  */
  
  /*
  float turn_r[135], turn_v[135], turn_l[135];
  turn_limit(135, turn_r, turn_v, turn_l);

  for(int i = 0; i < 135; i++)
  {
    printf("angle_%d: %f %f %f\n", i + 1, turn_r[i], turn_v[i], turn_l[i]);
  }
  */

  /*
  float distance_0 = speed_up_down_distance(0.0, 10.0, 1.0, 0.1);
  float distance_1 = speed_up_down_distance(10.0, 0.0, 1.0, 0.1);
  float distance_2 = speed_up_down_distance(2.0, 2.0, 1.0, 0.1);

  printf("distance = %f, %f, %f\n", distance_0, distance_1, distance_2);
  */

  float points[8][2] = {{0, 0}, {0, 50}, {0, 100}, {100, 100}, {100, 0}, {10, 0}, {10, 10}, {20, 10}};

  point_list* a;
  a = (point_list*) malloc(sizeof(point_list));
  a->list = (point*) malloc(sizeof(point) * 8);
  a->point_num = 8;
  //printf("%p\r\n", a->list);

  for(int i = 0; i < 8; i++)
  {
    a->list[i].x = points[i][0];
    a->list[i].y = points[i][1];
    //printf("%f , %f\r\n", a->list[i].x, a->list[i].y);
  }

  seg_list* s = points_2_segment(a, 8.0, 0.5, 0.1);

  Print_seg_list(s);

  point_list* aa = segment_rebuild_with_speed(s, 0.5, 0.1);
  Save_PointFile(aa);
  

  if(a) 
  {
    if(a->list) free(a->list);
    free(a);
  }
  if(aa) 
  {
    if(aa->list) free(aa->list);
    free(aa);
  }
  return ;
}

// 输出结果需要free
point_list* get_arc_point(point* start_point, float circle_r, float start_yaw,  float end_yaw, float speed)
{
  float yaw_step = 1 * Coff_Degree2Rad;     // 每次变化1度，可以修改

  float yaw_a = start_yaw;
  float yaw_b = end_yaw;
  float x0 = start_point->x;
  float y0 = start_point->y;
  float diff_yaw_ab = yaw_b - yaw_a;
  float current_yaw;
  if (diff_yaw_ab > pi)
  { 
    diff_yaw_ab = diff_yaw_ab - 2 * pi ;
  }
  else if (diff_yaw_ab < -pi)
  {  
    diff_yaw_ab = diff_yaw_ab + 2 * pi ;
  }
	
	
  float sub_line_length = yaw_step * circle_r;
	
  int step_num = floor(fabs(diff_yaw_ab) / yaw_step) - 1;

  if (step_num <= 0) return NULL;

  point_list* points = (point_list*) malloc(sizeof(point_list));
  points->list = (point*) malloc(sizeof(point) * step_num);
  points->point_num = step_num;
	
  if (diff_yaw_ab < 0.0) yaw_step = -yaw_step;
  for(int i = 0; i < step_num; i++)
  {
    current_yaw = yaw_a + yaw_step * (i + 1);
    if (current_yaw > pi) 
    {
      current_yaw = current_yaw - 2 * pi ;
    }
    else if  (current_yaw < -pi)
    {  
      current_yaw = current_yaw + 2 * pi ;
    }
    x0 += sub_line_length * cos(current_yaw);
    y0 += sub_line_length * sin(current_yaw);
    points->list[i].x = x0;
    points->list[i].y = y0;
    points->list[i].speed = speed;
  }
  return points;
}


//--------------------------
void turn_limit(float max_diff_yaw_degree, float* turn_r, float* turn_v, float* turn_l)
{
	float M = 260;    // kg
	float V = 1;      // m/s
	float R = 5;      // m
	float F_std = M * (V * V) / R;
	float L = 5;      // m
	float l = L;
	int support_max_diff_yaw_degree = floor(max_diff_yaw_degree); // degree 
  int i;
  for(i = 1; i <= support_max_diff_yaw_degree; i++)
  {
    turn_r[i - 1] = l / tan((float)i * 0.5 * Coff_Degree2Rad);  // 建议标准转弯半径
    turn_v[i - 1] = sqrt( F_std * turn_r[i - 1] / M);           // 建议标准转弯速度

    // 【转弯】限定最小转弯半径
    if (turn_r[i - 1] < R)
    {
	    turn_r[i - 1] = R;
	    turn_v[i - 1] = sqrt( F_std * turn_r[i - 1] / M);
	    turn_l[i - 1] = turn_r[i - 1] * tan((float)i * 0.5 * Coff_Degree2Rad) ; // 建议标准转弯提前距离
    }
    else
    {
      turn_l[i - 1] = l;     
    }
  }
}

// (start_speed, end_speed] , speed_up_down_distance.m的一部分，速度列表没有计算
float speed_up_down_distance(float start_speed, float end_speed, float speed_acc, float tick)
{
  int times;
  float distance = 0;
  float speed_step, cur_speed;
  times = round( fabs(end_speed - start_speed) / (speed_acc * tick) );
  speed_step = speed_acc * tick;
  if((end_speed - start_speed) < 0.0) speed_step = -speed_step;

  cur_speed = start_speed;
  for(int i = 1; i <= times; i++)
  {
    cur_speed += speed_step;
    distance += tick * cur_speed;
  }
  return distance;
}

float get_max_speed_of_distance(float speed_a, float speed_acc, float distance)
{
  return sqrt((speed_acc * 2 * distance) + (speed_a * speed_a));
}

float get_min_distance_of_two_speed_point(float speed_a, float speed_b, float speed_acc)
{
  float time = fabs( (speed_a-speed_b) / speed_acc );
  return (speed_a + speed_b) * 0.5 * time;
}


int speed_limit_rebuild(float* points_speed_limit, float* seg_distance, float speed_acc, int seg_num)
{
  float* speed_limit = points_speed_limit;
  float* sub_distance = seg_distance;
  int right_num = 0, debug_try_num = 0, i;
  while((right_num < seg_num) && (debug_try_num < seg_num))
  {
    right_num = 0;

    for(i = 0; i < seg_num; i++)
    {
      float min_distance = get_min_distance_of_two_speed_point(speed_limit[i], speed_limit[i + 1], speed_acc);

      if (round(min_distance * 10.0) > round(sub_distance[i] * 10.0))
      {
        // 降低较高的限制速度
        int higher_index = i;
        if (speed_limit[i] < speed_limit[i + 1])
        {
          higher_index = i + 1;
        }
        float lower_speed = min(speed_limit[i], speed_limit[i + 1]);
        float high_speed_limit = get_max_speed_of_distance(lower_speed, speed_acc, sub_distance[i]);
        speed_limit[higher_index] = high_speed_limit;
        debug_try_num += 1;
        break;
      }
      else
      {  
        right_num = right_num + 1;
      }
    }
  }
  return 0;
}


//--------------------------
// 输出结果需要free
point_list* get_line_run_point_speed(point* start_point, point* end_point, float start_speed, float end_speed, float speed_acc, float max_line_speed, float tick, float line_length)
{
  float distance_interg = 0.0;
  float distance_b = 0.0; 
  float dx = end_point->x - start_point->x;
  float dy = end_point->y - start_point->y;  
  float current_speed = start_speed; 
  float last_speed = current_speed; 
  int i, times_a = 0;

  while ((distance_interg + distance_b) < line_length)
  {
    last_speed = current_speed;
    current_speed = current_speed + speed_acc * tick;
    current_speed = min(current_speed, max_line_speed);
    
    // 路程积分
    distance_interg = distance_interg + current_speed * tick;    
    distance_b = speed_up_down_distance(current_speed, end_speed, speed_acc, tick);
    times_a += 1;
  }
  //printf("times_a %d, %f, %f\r\n", times_a, current_speed, last_speed);

  int times_b = round( fabs(end_speed - last_speed) / (speed_acc * tick) );

  //printf("times_b %d, %f, %f\r\n", times_b, end_speed, line_length);
  //----------------------------------------------------------------------
  assert((times_a + times_b) > 0);
  point_list* points = (point_list*) malloc(sizeof(point_list));
  points->list = (point*) malloc(sizeof(point) * (times_a + times_b));
  points->point_num = times_a + times_b;
  //----------------------------------------------------------------------

  float start_x = start_point->x;
  float start_y = start_point->y;
  distance_interg = 0.0;
  current_speed = start_speed; 
  for(i = 0; i < times_a; i++)
  {
    points->list[i].x = start_x + dx * distance_interg / line_length;
    points->list[i].y = start_y + dy * distance_interg / line_length;
    points->list[i].speed = current_speed;
    points->list[i].rsv = 0;   // useless

    current_speed = current_speed + speed_acc * tick;
    current_speed = min(current_speed, max_line_speed);
    distance_interg = distance_interg + current_speed * tick; 
  }
  //printf("s1, %f, %f, %f\r\n", max_line_speed, speed_acc, tick);

  //----------------------------------------------------------------------
  float speed_step = speed_acc * tick;
  if((end_speed - last_speed) < 0.0) speed_step = -speed_step;

  //printf("s2, %f, %f, %f\r\n", speed_step, end_speed, last_speed);

  start_x = points->list[times_a - 1].x;
  start_y = points->list[times_a - 1].y;
  distance_interg = 0.0;
  current_speed = last_speed; 

  //printf("s3, %f, %f, %f\r\n", start_x, start_y, current_speed);
  for(i = times_a; i < (times_a + times_b); i++)
  {
    current_speed += speed_step;
    distance_interg += tick * current_speed;

    points->list[i].x = start_x + dx * distance_interg / line_length;
    points->list[i].y = start_y + dy * distance_interg / line_length;
    points->list[i].speed = current_speed;
    points->list[i].rsv = 0;   // useless
  }
  //printf("s4, %d, %f, %f , %f\r\n", points->point_num, points->list[i - 1].x, points->list[i - 1].y, points->list[i - 1].speed);
  return points;
}



// 输出结果需要free
seg_list* points_2_segment(point_list* points, float max_speed, float max_speed_acc, float tick)
{
  // 模拟用户配置的值
  float global_speed_limit = max_speed; // 车体行驶的最大速度，m/s
  float global_speed_acc_limit = max_speed_acc; // 车体加速度的限定值，m/ss
  float global_time_tick = tick;

  // ---- 内部参数经验值 ----
  float param_max_turn_angle_degree = 135.0; // [-135, 135]
  float param_min_turn_angle_degree = -param_max_turn_angle_degree;
  float param_min_segment_length = 9.999999; // 最小线段长度  
  float param_min_turn_angle_to_circle_model = 4.0; // 4度以下不做圆弧模型处理

  // ---- 计算转弯限定值 ----
  int angle_num = floor(param_max_turn_angle_degree);
  float* turn_r = (float*) malloc(sizeof(float) * angle_num);
  float* turn_v = (float*) malloc(sizeof(float) * angle_num);
  float* turn_l = (float*) malloc(sizeof(float) * angle_num);
  turn_limit(param_max_turn_angle_degree, turn_r, turn_v, turn_l);

  // ---- 检查线段数量、长度 ----
  int i, point_num = points->point_num;
  int segment_num = points->point_num - 1;
  if (segment_num < 1)
  {
    if(turn_r) free(turn_r);
    if(turn_v) free(turn_v);
    if(turn_l) free(turn_l);

    printf("segment_num error!\r\n");
    return NULL;
  }
  int error_segment_num = 0;
  
  float* segment_length = (float*) malloc(sizeof(float) * segment_num);
  float* segment_yaw = (float*) malloc(sizeof(float) * segment_num);
  for (i = 0; i < segment_num; i++)
  {
    float dx = points->list[i + 1].x- points->list[i].x;
    float dy = points->list[i + 1].y- points->list[i].y;
    segment_length[i] = sqrt((dx * dx) + (dy * dy));
    segment_yaw[i] = atan2(dy, dx);
    if (segment_length[i] < param_min_segment_length)
    {
      error_segment_num = error_segment_num + 1;
    }
  }

  if (error_segment_num > 0)
  {

    if(turn_r) free(turn_r);
    if(turn_v) free(turn_v);
    if(turn_l) free(turn_l);
    if(segment_length) free(segment_length);
    if(segment_yaw) free(segment_yaw);
    printf("segment_length error 01 !\r\n");
    return NULL;
  }

  // ---- 检查线段与线段的航向角差值 ----
  float* seg_diff_yaw = NULL;
  int* seg_diff_angle = NULL;
  float* points_speed_limit = NULL;
  int* abs_seg_diff_angle = NULL;
  float* points_turn_l = NULL;
  float*  points_turn_arc_length = NULL;
  float*  points_turn_r = NULL;
  float*  segment_turn_l = NULL;
  float*  segment_length_of_run_direct = NULL;

  //printf("segment_num = %d\r\n", segment_num);

  if (segment_num >= 2)
  {
    int error_seg_diff_yaw_num = 0;
    seg_diff_yaw = (float*) malloc(sizeof(float) * (segment_num - 1));
    seg_diff_angle = (int*) malloc(sizeof(int) * (segment_num - 1));

    for (i = 0; i < (segment_num - 1); i++)
    {
      seg_diff_yaw[i] = segment_yaw[i + 1] - segment_yaw[i];
      if (seg_diff_yaw[i] > pi)
      { 
        seg_diff_yaw[i] = seg_diff_yaw[i] - 2 * pi ;
      }
      else if  (seg_diff_yaw[i] < -pi)  
      {
        seg_diff_yaw[i] = seg_diff_yaw[i] + 2 * pi ;
      }
      seg_diff_angle[i] = round(seg_diff_yaw[i] * Coff_Rad2Degree);
      if ((seg_diff_angle[i] < (int)param_min_turn_angle_degree) || (seg_diff_angle[i] > (int)param_max_turn_angle_degree)) 
      {   
        error_seg_diff_yaw_num = error_seg_diff_yaw_num + 1;
      }
    }
    
    if (error_seg_diff_yaw_num > 0)
    {
      if(turn_r) free(turn_r);
      if(turn_v) free(turn_v);
      if(turn_l) free(turn_l);
      if(segment_length) free(segment_length);
      if(segment_yaw) free(segment_yaw);      
      if(seg_diff_yaw) free(seg_diff_yaw);
      if(seg_diff_angle) free(seg_diff_angle);

      printf("seg_diff_yaw error!\r\n");
      return NULL;
    }      
  
    // ---- 计算所有点的限速值 ----
    points_speed_limit = (float*) calloc(sizeof(float) * point_num, 1); // 初始化各点的限速值，起始点和终点速度都是0
    abs_seg_diff_angle = (int*) malloc(sizeof(int) * (segment_num - 1));

    //abs_seg_diff_angle = abs(seg_diff_angle);
    for(i = 0; i < (segment_num - 1); i++) abs_seg_diff_angle[i] = abs(seg_diff_angle[i]);

    //for i = 2 : (point_num - 1)
    for (i = 1 ; i < (point_num - 1); i++)
    {
      int temp_seg_diff_angle = abs_seg_diff_angle[i - 1];
      if (temp_seg_diff_angle < 1)  // 直线的情况
      {
        points_speed_limit[i] = global_speed_limit;
      }
      else
      {
        points_speed_limit[i] = min(turn_v[temp_seg_diff_angle - 1], global_speed_limit);
      }
    }
    
    // ---- 计算所有点的转弯提前长度 ----
    points_turn_l = (float*) calloc(sizeof(float) * point_num, 1); // 初始化各点的提前转弯长度，起始点和终点长度都是0
    points_turn_arc_length = (float*) calloc(sizeof(float) * point_num, 1);
    points_turn_r = (float*) calloc(sizeof(float) * point_num, 1);

    //for i = 2 : (point_num - 1)
    for (i = 1 ; i < (point_num - 1); i++)
    {
      int temp_seg_diff_angle = abs_seg_diff_angle[i - 1];
      if (temp_seg_diff_angle >= param_min_turn_angle_to_circle_model)  
      {
        points_turn_l[i] = turn_l[temp_seg_diff_angle - 1];
        points_turn_arc_length[i] = turn_r[temp_seg_diff_angle - 1] * temp_seg_diff_angle * Coff_Degree2Rad;
        points_turn_r[i] = turn_r[temp_seg_diff_angle - 1];
      }
    }
    
    
    // ---- 检查【线段长度】是否大于【转弯提前长度】 ----
    segment_turn_l = (float*) calloc(sizeof(float) * segment_num, 1);
    error_segment_num = 0;
    //for i = 1:segment_num
    for (i = 0; i < segment_num; i++)
    {
      segment_turn_l[i] = points_turn_l[i] + points_turn_l[i + 1];
      if (segment_length[i] < segment_turn_l[i])
      {
        error_segment_num = error_segment_num + 1;
      }
    }
    if (error_segment_num > 0)
    {
      if(turn_r) free(turn_r);
      if(turn_v) free(turn_v);
      if(turn_l) free(turn_l);
      if(segment_length) free(segment_length);
      if(segment_yaw) free(segment_yaw);      
      if(seg_diff_yaw) free(seg_diff_yaw);
      if(seg_diff_angle) free(seg_diff_angle);

      if(points_speed_limit)     free(points_speed_limit);
      if(abs_seg_diff_angle)     free(abs_seg_diff_angle);
      if(points_turn_l)          free(points_turn_l);
      if(points_turn_arc_length) free(points_turn_arc_length);
      if(points_turn_r)          free(points_turn_r);
      if(segment_turn_l)         free(segment_turn_l);
      printf("segment_length error 02 !");
      return NULL;
    }    
    
    // ---- 计算【线段长度】-【转弯提前长度】后的长度 ----
    segment_length_of_run_direct = (float*) malloc(sizeof(float) * segment_num);
    for (i = 0; i < segment_num; i++)
    {
      segment_length_of_run_direct[i] = segment_length[i] - segment_turn_l[i];
    }
    
    // ---- 根据实际的【线段长度】和【最大加速度】，对各点的限速做重新计算 ----
    speed_limit_rebuild(points_speed_limit, segment_length_of_run_direct, global_speed_acc_limit, segment_num);
  } // ---- end of [segment_num >= 2] ----    

  seg_list* segments = (seg_list*) malloc(sizeof(seg_list));
  // ---- 将【单/多线段路径】微分 ----
  if (segment_num == 1)
  {
    segments->seg_num = 1;
    segments->list = (seg_param*) malloc(sizeof(seg_param) * 1);
    segments->list[0].type = 1; // 1 - 直线 ，2 - 弧线
    segments->list[0].length = segment_length[0];
    segments->list[0].speed_limit = global_speed_limit;
    segments->list[0].start_yaw = segment_yaw[0];
    segments->list[0].end_yaw = segment_yaw[0];   
    segments->list[0].r = 0;

    segments->list[0].p_start.x = points->list[0].x;   
    segments->list[0].p_start.y = points->list[0].y;   
    segments->list[0].p_end.x = points->list[1].x;   
    segments->list[0].p_end.y = points->list[1].y;    
    segments->list[0].p_start.speed = 0;
    segments->list[0].p_end.speed = 0;
  }
  else // ---- 多条线段 ----
  {
    int mix_seg_index = 0;
    for (i = 0; i < segment_num; i++)
    {
      if (segment_length_of_run_direct[i] > 0)  mix_seg_index += 1; // 有直线距离
      if (points_turn_l[i + 1] > 0) mix_seg_index += 1;                 // 有弧线
    }
    segments->seg_num = mix_seg_index;
    segments->list = (seg_param*) malloc(sizeof(seg_param) * mix_seg_index);
    int j = 0;
    for (i = 0; i < segment_num; i++)
    {
      if (segment_length_of_run_direct[i] > 0)
      {
        segments->list[j].type = 1; // 1 - 直线 ，2 - 弧线
        segments->list[j].length = segment_length_of_run_direct[i];
        segments->list[j].speed_limit = global_speed_limit;
        segments->list[j].start_yaw = segment_yaw[i];
        segments->list[j].end_yaw = segment_yaw[i];
        segments->list[j].r = 0;

        segments->list[j].p_start.x = points->list[i].x + points_turn_l[i] * (points->list[i + 1].x - points->list[i].x) / segment_length[i];  
        segments->list[j].p_start.y = points->list[i].y + points_turn_l[i] * (points->list[i + 1].y - points->list[i].y) / segment_length[i];  
        segments->list[j].p_end.x = points->list[i + 1].x + points_turn_l[i + 1] * (points->list[i].x - points->list[i + 1].x) / segment_length[i];  
        segments->list[j].p_end.y = points->list[i + 1].y + points_turn_l[i + 1]  * (points->list[i].y - points->list[i + 1].y) / segment_length[i];  
        segments->list[j].p_start.speed = points_speed_limit[i];
        segments->list[j].p_start.rsv = 0;
        segments->list[j].p_end.speed = points_speed_limit[i + 1];
        segments->list[j].p_end.rsv = 0;
        j += 1;
      }
      if (points_turn_l[i + 1] > 0)
      {
        segments->list[j].type = 2; // 1 - 直线 ，2 - 弧线
        segments->list[j].length = points_turn_arc_length[i + 1]; // 弧线长度
        segments->list[j].speed_limit = points_speed_limit[i + 1];
        segments->list[j].start_yaw = segment_yaw[i];
        segments->list[j].end_yaw = segment_yaw[i + 1];
        segments->list[j].r = points_turn_r[i + 1];

        segments->list[j].p_start.x = points->list[i + 1].x + points_turn_l[i + 1] * (points->list[i].x - points->list[i + 1].x) / segment_length[i];  
        segments->list[j].p_start.y = points->list[i + 1].y + points_turn_l[i + 1] * (points->list[i].y - points->list[i + 1].y) / segment_length[i];  
        segments->list[j].p_end.x = points->list[i + 1].x + points_turn_l[i + 1] * (points->list[i + 1 + 1].x - points->list[i + 1].x) / segment_length[i + 1];  
        segments->list[j].p_end.y = points->list[i + 1].y + points_turn_l[i + 1]  * (points->list[i + 1 + 1].y - points->list[i + 1].y) / segment_length[i + 1];  
        segments->list[j].p_start.speed = points_speed_limit[i + 1];
        segments->list[j].p_start.rsv = 0;
        segments->list[j].p_end.speed = points_speed_limit[i + 1];
        segments->list[j].p_end.rsv = 0;
        j += 1; 
      }
    }
  }

  if(turn_r) free(turn_r);
  if(turn_v) free(turn_v);
  if(turn_l) free(turn_l);
  if(segment_length) free(segment_length);
  if(segment_yaw) free(segment_yaw);      
  if(seg_diff_yaw) free(seg_diff_yaw);
  if(seg_diff_angle) free(seg_diff_angle);

  if(points_speed_limit)     free(points_speed_limit);
  if(abs_seg_diff_angle)     free(abs_seg_diff_angle);
  if(points_turn_l)          free(points_turn_l);
  if(points_turn_arc_length) free(points_turn_arc_length);
  if(points_turn_r)          free(points_turn_r);
  if(segment_turn_l)         free(segment_turn_l);
  if(segment_length_of_run_direct) free(segment_length_of_run_direct);

  return segments;
}

void Print_seg_list(seg_list* segs)
{
  printf("=================================\r\n");
  printf("seg_num = %d\r\n", segs->seg_num);
  for(int i = 0; i < segs->seg_num; i++)
  {
    printf("(No. %d) ------------------\r\n", i);
    printf("type :        %d\r\n", segs->list[i].type);
    printf("length :      %f\r\n", segs->list[i].length);
    printf("speed_limit : %f\r\n", segs->list[i].speed_limit);
    printf("start_yaw :   %f\r\n", segs->list[i].start_yaw);
    printf("end_yaw :     %f\r\n", segs->list[i].end_yaw);
    printf("r :           %f\r\n", segs->list[i].r);
    printf("start :       (%f , %f), v = %f\r\n", segs->list[i].p_start.x, segs->list[i].p_start.y, segs->list[i].p_start.speed);
    printf("end :         (%f , %f), v = %f\r\n", segs->list[i].p_end.x, segs->list[i].p_end.y, segs->list[i].p_end.speed);
  }
  printf("=================================\r\n");
}

// 输出结果需要free
point_list* segment_rebuild_with_speed(seg_list* list, float speed_acc, float tick)
{
  int segment_num = list->seg_num, i;
  point_list** p_list = calloc(sizeof(point_list*) ,segment_num);
  point_list* all = NULL;
  int num = 0;

  for(i = 0; i < segment_num; i++)
  {
    if (list->list[i].type == 1)
    {
      p_list[i] = get_line_run_point_speed(&(list->list[i].p_start), &(list->list[i].p_end), list->list[i].p_start.speed , list->list[i].p_end.speed, 
                    speed_acc, list->list[i].speed_limit, tick, list->list[i].length);
      num += p_list[i]->point_num;
    }
    else if (list->list[i].type == 2)
    {
      p_list[i] = get_arc_point(&(list->list[i].p_start), list->list[i].r, list->list[i].start_yaw,  list->list[i].end_yaw, list->list[i].speed_limit);
      num += p_list[i]->point_num;
    }
    else
    {
      printf("segment_param.type error !\r\n");
    }

    Print_point_list(p_list[i]);
  }

  // ---- 合成一条线 ----
  all = (point_list*) malloc(sizeof(point_list));
  all->list = (point*) malloc(sizeof(point) * num);
  all->point_num = num;
  num = 0;

  for(i = 0; i < segment_num; i++)
  {
    if(p_list[i] != NULL)
    {
      int sub_num = p_list[i]->point_num;
      memcpy(all->list + num, p_list[i]->list, sizeof(point) * sub_num);
      num += sub_num;
      free(p_list[i]->list);
      free(p_list[i]);
    }
  }
  Print_point_list(all);

  free(p_list);
  return all;
}
  

void Print_point_list(point_list* points)
{
  printf("=================================\r\n");
  printf("point_num = %d\r\n", points->point_num);
  /*
  for(int i = 0; i < points->point_num; i++)
  {

  }*/
  point* p = points->list + 0;
  printf("[%f, %f, %f]\r\n", p->x, p->y, p->speed);
  p = points->list + points->point_num - 1;
  printf("[%f, %f, %f]\r\n", p->x, p->y, p->speed);
}

void Save_PointFile(point_list* points)
{
  char file_name[512];
  struct timespec time;
  struct tm nowtime;
  clock_gettime(CLOCK_REALTIME, &time);
  localtime_r(&time.tv_sec, &nowtime);

  sprintf(file_name, "points_%04d_%02d%02d_%02d%02d%02d.txt", nowtime.tm_year + 1900, nowtime.tm_mon + 1, nowtime.tm_mday, nowtime.tm_hour, nowtime.tm_min, nowtime.tm_sec);
  
  FILE* fd = fopen(file_name,"wt+");
  if(fd == NULL)
  {
    printf("Open Points File Error !\n");
    return;
  }

  fprintf(fd, "=================================\r\n");
  fprintf(fd, "point_num = %d\r\n", points->point_num);
  point* p = points->list + 0;
  for(int i = 0; i < points->point_num; i++)
  {
    fprintf(fd, "%f\t%f\t%f\t\r\n", p->x, p->y, p->speed);
    p++;
  }
  
  printf("save points to %s\r\n", file_name);
  if(fd != NULL)
  {
    fclose(fd);
  }
}

//----------------------------------------------------------------------------------

int VEHICLE_10_Run(MODULE_CORE_PARAM * param, point_list* pl, GPSINFO* gps, int fd_car, FILE* log)
{
  double control_cycle = 0.1; //控制周期，100ms
  static int counter = 0;
  static int zero_speed_counter = 0;
  int ret = 0;
  int cur_ref_index;

  double yaw0;
  if(VEHICLE_RESET_Extern != 0)
  {
    VEHICLE_RESET_Extern = 0;
    counter = 0;
    zero_speed_counter = 0;
  }

  if(PID_Flag)
  {
    PID_Flag = 0;

#if (1)
    if(log != NULL)
    {
      if ((gps->FixMode != 4) || (gps->AVR_FixMode != 3))
        fprintf(log, "++++ Error: GPS FixMode (%d , %d) ++++ \n", gps->FixMode, gps->AVR_FixMode);
    }
#endif

    cur_ref_index = param->cur_ref_index;
  
    Get_Coordinate(param->current_location, gps->latitude_InM_RTK_Fixed, gps->longitude_InM_RTK_Fixed, gps->Yaw_RTK_Fixed, param->coo);

    double x0 = param->current_location->x;
    double y0 = param->current_location->y;
    yaw0 = param->current_location->direction;


    // ---- 找到最近点作为参照点 ----
    int max_ref_index = min(cur_ref_index + param->ref_point_ahead_num, pl->point_num);
    point* ref_point = pl->list + cur_ref_index;
    float min_distance = ((x0 - ref_point->x) * (x0 - ref_point->x)) + ((y0 - ref_point->y) * (y0 - ref_point->y));
    int min_distance_index = cur_ref_index;
    for(int i = cur_ref_index + 1; i < max_ref_index; i++)
    {
      ref_point = pl->list + i;
      float tmp = ((x0 - ref_point->x) * (x0 - ref_point->x)) + ((y0 - ref_point->y) * (y0 - ref_point->y));
      if(tmp < min_distance)
      {
        min_distance = tmp;
        min_distance_index = i;
      }
    }
    min_distance  = sqrt(min_distance);
    cur_ref_index = min_distance_index;
    param->cur_ref_index = min_distance_index;


    //+++++++++++ 计算ref线段方程和垂线方程 +++++++++++++
    ref_point = pl->list + cur_ref_index;
    point* ref_point_p1 = ref_point + 1;

    float ref_yaw = atan2(ref_point_p1->y - ref_point->y, ref_point_p1->x - ref_point->x);
    float ref_speed = ref_point_p1->speed;
    
    float dx = ref_point_p1->x - ref_point->x;
    float dy = ref_point_p1->y - ref_point->y;
    float Kx, Ky, v_Kx, v_Ky, b, v_b;
    if (dx == 0.0)
    {
      Kx = 1.0;
      Ky = 0.0;
      v_Kx = 0.0;
      v_Ky = 1.0;
    }
    else if (dy == 0.0)
    {
      Kx = 0.0;
      Ky = 1.0;
      v_Kx = 1.0;
      v_Ky = 0.0; 
    }
    else
    {
      Ky = 1.0;
      Kx = dy / dx;
      v_Kx = tan(atan(Kx) + pi * 0.5);
      v_Ky = 1.0; 
    }
    b = Ky * ref_point->y - Kx * ref_point->x;
    v_b = v_Ky * y0 - v_Kx * x0;

    //计算当前点到规定路径的垂线距离

    double kx1,ky1,kx2,ky2,b1,b2;
    kx1 = Kx;
    ky1 = Ky;
    b1 = b;
    kx2 = v_Kx;
    ky2 = v_Ky;
    b2 = v_b;

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
    
    double distance = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
    double diff = ky1 * y0 - (kx1 * x0 + b1);

    float x3 = ref_point_p1->x;
    float y3 = ref_point_p1->y;
    float yaw2 = atan2(y3 - y0, x3 - x0);
    
    float diff_yaw3 = yaw2 - ref_yaw;
    if (diff_yaw3 > pi)
    {
      diff_yaw3 = diff_yaw3 - 2.0 * pi;
    }
    else if  (diff_yaw3 <= -pi)
    {
      diff_yaw3 = diff_yaw3 + 2.0 * pi;
    }     
    
    if (diff_yaw3 < 0.0)
    {
      // ---- 右拐 ----
      distance = -distance;
    }

    
    //---- 特殊情况处理 ----//
    if(fabs(distance) > param->max_away_from_line)
    {
      printf("Beyond max error! Error Code %d.\r\n", 0x0001);
      param->left_speed = 0;
      param->right_speed = 0;
      param->error_flag = 1;
      SetMotoSpeedAsync(param->left_speed * 1000.0d, param->right_speed * 1000.0d);
      ret = -1;

      counter = 0;
      zero_speed_counter = 0;
      return ret;
    }

    // 提前10个点，将期望速度设置为0
    if((cur_ref_index + 10) >= pl->point_num)  
    {
      ref_speed = 0.0;
    }

    // 车体前进方向加减速控制，实现匀加速和匀减速过程
    int fix_car_speed = param->car_speed * 10000.0;
    int fix_expected_speed = ref_speed * 10000.0; 
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
      SetMotoSpeedAsync(param->left_speed * 1000.0d, param->right_speed * 1000.0d);

      counter = 0;
      zero_speed_counter = 0;
      return ret;
    }
    
    // 运行距离积分
    param->run_distance += (param->car_speed * control_cycle);

    // 航向角处理，航向角差在目前的方案中限定在(-PI，PI] (*重要改动*)
    double diff_yaw = yaw0 - ref_yaw;
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

    float abs_speed = fabs(param->car_speed); 
    //----------------------//
    param->last_error = param->error;
    param->error = distance;
    param->error_sum += distance;
    if(param->error_sum > param->max_error_sum) param->error_sum = param->max_error_sum;
    else if(param->error_sum < param->min_error_sum) param->error_sum = param->min_error_sum;

#if (1)
    if(abs_speed > 1.0) // 差速等比缩小，防止绝对差速过大
    {
      param->error = param->error / abs_speed;
    } 
#endif

    double pid_out = param->P * param->error + param->I * param->error_sum + param->D * (param->error - param->last_error) + (-1.0) * diff_yaw * param->yaw_P;
    pid_out = MAX(pid_out, -1.0);
    pid_out = MIN(pid_out,  1.0);


    double diff_rate = pid_out * param->max_diff_rate;
    
    if(abs_speed > 1.0) // 差速等比缩小，防止绝对差速过大
    {
      diff_rate = diff_rate / abs_speed;
    }  

    param->left_speed = param->car_speed * (1.0 - 0.5 * diff_rate);
    param->right_speed = param->car_speed * (1.0 + 0.5 * diff_rate);

    if((counter & 15) == 0)
    {
      printf("\r\n  curr_x = %lf | %f, curr_y = %lf | %f, curr_yaw = = %lf | %f [ error = %lf , pid_out = %lf] [l = %lf , r = %lf, %c] dis = %lf total = %lf\r\n", \
        param->current_location->x, ref_point->x, param->current_location->y,  ref_point->y, yaw0, ref_yaw, \
        param->error, pid_out, param->left_speed, param->right_speed, \
        (param->left_speed > param->right_speed)?'R':'L', fabs(distance), param->run_distance);
    }

    SetMotoSpeedAsync(param->left_speed * 1000.0d, param->right_speed * 1000.0d);

    if(log != NULL)
    {
      //fprintf(log, "cur_x \t cur_y \t cur_yaw \t line_x \t line_y \t line_yaw \t l_speed \t r_speed \n");
      fprintf(log, "%lf \t %lf \t %lf \t %lf \t %lf \t %lf \t %lf \t %lf \n", \
        param->current_location->x, param->current_location->y, param->current_location->direction, \
        x1, y1, ref_yaw, \
        param->left_speed, param->right_speed);
    }

    counter += 1;

    UpdateModBusRegs((int) (param->current_location->x * 100.0d), (int) (param->current_location->y * 100.0d), (short) yaw0 , (short)(param->car_speed * 10.0));
  }
  return ret;
}
