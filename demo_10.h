#ifndef _demo_10_h_
#define _demo_10_h_

#include "module_core.h"

// --------------------------------
typedef struct
{
  float x;
  float y;
  float speed;
  float rsv; // 弧度
}point;

typedef struct
{
  int point_num;
  point* list;
}point_list;

typedef struct
{
  int type;
  float length;
  float speed_limit;
  float start_yaw;
  float end_yaw;
  float r;
  point p_start;
  point p_end;  
}seg_param;

typedef struct
{
  int seg_num;
  seg_param* list;
}seg_list;

// --------------------------------
point_list* get_arc_point(point* start_point, float circle_r, float start_yaw,  float end_yaw, float speed);
void turn_limit(float max_diff_yaw_degree, float* turn_r, float* turn_v, float* turn_l);
float speed_up_down_distance(float start_speed, float end_speed, float speed_acc, float tick);
float get_max_speed_of_distance(float speed_a, float speed_acc, float distance);
float get_min_distance_of_two_speed_point(float speed_a, float speed_b, float speed_acc);
int speed_limit_rebuild(float* points_speed_limit, float* seg_distance, float speed_acc, int seg_num);
point_list* get_line_run_point_speed(point* start_point, point* end_point, float start_speed, float end_speed, float speed_acc, float max_line_speed, float tick, float line_length);
seg_list* points_2_segment(point_list* points, float max_speed, float max_speed_acc, float tick);
point_list* segment_rebuild_with_speed(seg_list* list, float speed_acc, float tick);
void Print_seg_list(seg_list* segs);
void Print_point_list(point_list* points);
void Save_PointFile(point_list* points);
int VEHICLE_10_Run(MODULE_CORE_PARAM * param, point_list* pl, GPSINFO* gps, int fd_car, FILE* log);
#endif
