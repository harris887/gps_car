#include <string.h>
#include <stdio.h>
#include <stdlib.h> 
#include <math.h>  

#ifndef PI
#define PI 3.1415926535898
#endif 

double Get_TwoPointDistance(double lati_01, double longti_01, double lati_02, double longti_02)
{
    double earth_r = 6371 * 1000.0;
    double lati_cycle_r = earth_r * cos((lati_01+lati_02) * (PI / 180.0) * 0.5 / 60.0 );//
    double dx = (longti_01 - longti_02) * (PI / 180.0) * (lati_cycle_r / 60.0);
    double dy = (lati_01 - lati_02) * (PI / 180.0) * (earth_r / 60.0);
    printf("lati_r = %lf, dx = %lf, dy = %lf \r\n", lati_cycle_r, dx, dy);
    return sqrt(dx * dx + dy * dy);
}

typedef unsigned char  u8;
typedef signed char    s8;
typedef unsigned short u16;
typedef signed short   s16;
typedef unsigned int   u32;
typedef signed int     s32;

u16 ModBus_CRC16_Calculate(u8 *aStr , u8 alen);

typedef struct
{
  u16 map_num;
  u16 map_index;
}MAP_LIST;

MAP_LIST MAP_List = {1, 1};
#define MAP_LIST_REG_BASE  0x200

typedef struct
{
  u16 map_start_reg;
  u16 map_reg_num;
}MAP_SUMMERY;

MAP_SUMMERY MAP_Summery = {0x400, 2 + 2 * 5};
#define MAP_SUMMERY_REG_BASE 0x220

typedef struct
{
  u16 Map_Type;            
  u16 Point_Num;           
  s16 Point_Coordinate_XY[5][2];  
}MAP_PARAM;

MAP_PARAM MAP_Param = {0, 5, 0,0, 0,20, 20,20, 20,0, 0,0};
#define MAP_PARAM_REG_BASE 0x400

typedef struct
{
  u16 longti_H;    
  u16 longti_L;
  u16 lati_H;          
  u16 lati_L; 
}ORIGIN_POINT;

// 浮点：116.353604, 40.039227
// hex : 0x42e8b50c , 0x4220282b
ORIGIN_POINT ORIGIN_Point = { 
  0x42e8, 0xb50c,
  0x4220, 0x282b,
};
#define ORIGIN_POINT_REG_BASE 0x28
//--------------------------------------------------------

#define TEST_CASE_ENUM   3 //0,1,2,3

#if (TEST_CASE_ENUM == 0)
#define TEST_STRUCT_NAME   MAP_List
#define REG_BASE           MAP_LIST_REG_BASE  
#elif (TEST_CASE_ENUM == 1)
#define TEST_STRUCT_NAME   MAP_Summery
#define REG_BASE           MAP_SUMMERY_REG_BASE  
#elif (TEST_CASE_ENUM == 2)
#define TEST_STRUCT_NAME   MAP_Param
#define REG_BASE           MAP_PARAM_REG_BASE  
#elif (TEST_CASE_ENUM == 3)
#define TEST_STRUCT_NAME   ORIGIN_Point
#define REG_BASE           ORIGIN_POINT_REG_BASE  
#endif

int Creat_Map(void)
{
  int rw_select = 1; // r-0, w-1
  u16 reg_base = REG_BASE;
  u16 reg_num = sizeof(TEST_STRUCT_NAME) / sizeof(u16);
  u16 crc;
  unsigned char buf[128];
  int index = 0;
  u16* reg_data = (u16*) &TEST_STRUCT_NAME;

  
  buf[index++] = 0x01;
  if(rw_select == 0) buf[index++] = 0x04;
  else if ((rw_select != 0) &&  (reg_num == 1)) buf[index++] = 0x06;
  else if ((rw_select != 0) &&  (reg_num > 1)) buf[index++] = 0x16;
  else 
  {
    printf("error 01 \r\n");
    return 0;
  }
  buf[index++] = (reg_base >> 8) & 0xFF;
  buf[index++] = reg_base & 0xFF;
  if((rw_select == 0) && (reg_num > 0) && (reg_num <= 32))
  {
    buf[index++] = (reg_num >> 8) & 0xFF;
    buf[index++] = reg_num & 0xFF;
    crc = ModBus_CRC16_Calculate(buf, index);
    buf[index++] = crc & 0xFF;
    buf[index++] = crc >> 8;
  }
  else if((rw_select != 0) &&  (reg_num == 1))
  {
    buf[index++] = (reg_data[0] >> 8) & 0xFF;
    buf[index++] = reg_data[0] & 0xFF;
    crc = ModBus_CRC16_Calculate(buf, index);
    buf[index++] = crc & 0xFF;
    buf[index++] = crc >> 8;
  }
  else if ((rw_select != 0) &&  (reg_num > 1) && (reg_num <= 32))
  {
    buf[index++] = (reg_num >> 8) & 0xFF;
    buf[index++] = reg_num & 0xFF;
    for(int i = 0; i < reg_num; i++) 
    {
      buf[index++] = (reg_data[i] >> 8) & 0xFF;
      buf[index++] = reg_data[i] & 0xFF;
    }
    crc = ModBus_CRC16_Calculate(buf, index);
    buf[index++] = crc & 0xFF;
    buf[index++] = crc >> 8;
  }
  
  //---- 打印到终端 ----//
  printf("\r\n");
  for(int i = 0; i < index; i++) 
  {
    printf("%02X ", buf[i]);
  }
  printf("\r\n");
}


double jw[2][2] = 
{
  {116.420973,40.048134},
  {116.44131,40.048852},
};
void main(void)
{
  //double dis = Get_TwoPointDistance(jw[0][1] * 60.0d, jw[0][0] * 60.0d, jw[1][1] * 60.0d, jw[1][0] * 60.0d);

  //printf("dis = %lf\r\n", dis);

  Creat_Map();
  return ;
}

/********************************************************************************
函数名称:u16 ModBus_CRC16_Calculate(unsigned char *aStr , unsigned char alen)
函数功能:计算发送数据 CRC 校验功能
********************************************************************************/
u16 ModBus_CRC16_Calculate(u8 *aStr , u8 alen)
{
  u16 xda,xdapoly;
  u8 i,j,xdabit;
  xda = 0xFFFF;
  xdapoly = 0xA001;	// (X**16 + X**15 + X**2 + 1)
  for(i=0;i<alen;i++) 
  {
    xda ^= aStr[i];
    for(j=0;j<8;j++)
    {
      xdabit = (u8)(xda & 0x01);
      xda >>= 1;
      if( xdabit ) xda ^= xdapoly;
    }
  }    
  return xda;
}
