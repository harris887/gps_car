//串口相关的头文件  
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
#include "nmea.h"
#include "uart.h"
#include "misc.h"   
#include "demo_02.h"
#include "demo_03.h"
#include "config.h"
#include "log.h"

typedef enum
{
  exe_name = 0,
  gps_com,
  moto_com,
  radio_com,
  log_option,
  param_num
}PARAM; 


void* Load_CmdList(const char* url);
int UART0_RX_Task(int fd);
int Remove_CompeleteFrame(char* base, int* base_len, char* frame, int frame_len, long time);
int Flush_InCompleteFrame(char* base, int* base_len, long time);
#define IN_COMPELETE_BUF_SIZE (1024*1024*16)
char InCompleteFrame[IN_COMPELETE_BUF_SIZE+1024];
int InCompleteFrameIndex = 0;
//-- 01 06 00 02 00 04 29 C9  -9600
//-- 01 06 00 02 00 05 E8 09  -19200 
//-- 01 06 00 02 00 06 A8 08  -38400
//-- 01 06 00 02 00 07 69 C8  -57600
//-- 01 06 00 02 00 08 29 CC  -115200
int main(int argc, char **argv)  
{  
  FILE* log = NULL;
  int busy_flag, sleep_ms = 0;
  //cmd_loop_all* handle;
  int bd;
  int loop_flag = 1;
  char keyboard_char = 0;
  //---- 串口相关 ----//	
  int fd_gps = 0, fd_car = 0, fd_radio = 0;                                         //文件描述符  
	int err;                                        //返回调用函数的状态 
  int comm_init_retry_num ; 

#if(0) //
  for(int i = 0; i < 360; i++)
  {
    printf("yaw = %d, rad = %lf \r\n", i, YAW2DIR(i));
  }
  return 0;
#endif

#if (SIMULATE_ENABLE) 
  if(argc >= 2)
  {
    if(atoi(argv[1]))
    {
      log = Log_Init();
    }
  }
#else //---- 打开串口 ----//
	if(argc < param_num)  
	{ 
		//printf("echo loopback test! \n"); 
		printf("Usage: %s gps_com moto_com radio_com [0:un_log, 1:log]\n",argv[0]); //  /dev/ttyUSB*
		return FALSE;  
	} 
  if(argc >= (log_option + 1))
  {
    if(atoi(argv[log_option]))
    {
      log = Log_Init();
    }
  }

  bd = 115200; //(argc>=5)?atoi(argv[4]):DEFAULT_UART_BD;
  comm_init_retry_num = 0;
  fd_gps = UART0_Open(fd_gps,argv[gps_com]);                    //打开串口，返回文件描述符  
	do
  {  
		err = UART0_Init(fd_gps, bd, 0, 8, 1, 'N');  
		//printf("Set Port Exactly!\n");  
    comm_init_retry_num += 1;
    sleep(1);
	}while((FALSE == err || FALSE == fd_gps) && (comm_init_retry_num < 3));  
  if(FALSE == err) 
  {
    printf("SerialPort [%s] Error !\n", argv[gps_com]);
    return FALSE;   
  }

  bd = 115200;
  comm_init_retry_num = 0;
  fd_car = UART0_Open(fd_car,argv[moto_com]);                    //打开串口，返回文件描述符  
	do
  {  
		err = UART0_Init(fd_car, bd, 0, 8, 1, 'N');  
		//printf("Set Port Exactly!\n");  
    comm_init_retry_num += 1;
    sleep(1);
	}while((FALSE == err || FALSE == fd_car) && (comm_init_retry_num < 3));  
  if(FALSE == err) 
  {
    printf("SerialPort [%s] Error !\n", argv[moto_com]);
    return FALSE;   
  }
  else
  {
    SetMotoSpeed(fd_car, 0, 0);
  }

  bd = 9600;
  comm_init_retry_num = 0;
  fd_radio = UART0_Open(fd_radio,argv[radio_com]);                    //打开串口，返回文件描述符  
	do
  {  
		err = UART0_Init(fd_radio, bd, 0, 8, 1, 'N');  
		//printf("Set Port Exactly!\n");  
    comm_init_retry_num += 1;
    sleep(1);
	}while((FALSE == err || FALSE == fd_radio) && (comm_init_retry_num < 3));  
  if(FALSE == err) 
  {
    printf("SerialPort [%s] Error !\n", argv[radio_com]);
    return FALSE;   
  }
#endif

  timer_init();

	//printf("BLOCK_STOP_MODE = %d\n", BLOCK_STOP_MODE);

  printf("fd_gps = 0x%X, fd_car = 0x%X, fd_radio = 0x%X\n", fd_gps, fd_car, fd_radio);
  printf("\n---- main loop ----\n");
#if (SIMULATE_ENABLE) 
#else
  printf("second  latitude  longitude altitude FixMode0 Yaw FixMode1 SVsInUse GGA AVR\n");
#endif
  DEMO_01_PARAM* demo_01 = NULL;
  DEMO_02_PARAM* demo_02 = NULL;
  DEMO_03_PARAM* demo_03 = NULL;
  int pro = 0;
  system(STTY_US TTY_PATH);
  //-------------------------------------------------------------//
  while(loop_flag)
  {
    static int sec_bk = 0;
    timer_check();
    keyboard_char = get_char();

#if (SIMULATE_ENABLE) 
#else
    UART0_RX_Task(fd_gps);
    MODBUS_UART_RX_Task(fd_radio);
    //---- parse NMEA ----//
    for(int i=0;i<NMEA_R_BUF_LEN;i++)
    {
      if(NMEA_R_BUF[i].Flag==2)
      {
        NMEA_parse(NMEA_R_BUF[i].BUF);
        NMEA_R_BUF[i].Flag=0;
      }
    }

    if(sec_bk != system_time_in_sec)
    {
      sec_bk = system_time_in_sec;
      //printf("second = %d \n", sec_bk);
      if(pro < 2)
        printf("\r%d  [ %lf %lf %f %d ] [ %f %d ] [ %d ]  [ %d  %d ]", sec_bk, gps.latitude_InM, gps.longitude_InM, gps.altitude, gps.FixMode, \
          gps.Yaw, gps.AVR_FixMode, gps.usedsatnum, gps.GGA_Num, gps.PTNL_AVR_Num);
      fflush(stdout);
    }
#endif
    
    static double lati_1 = 2406.239004, longti_1 = 6982.805081;
    switch(keyboard_char)
    {
    case 0: break;
    case 3: //ctrl+c
      {
        system(STTY_DEF TTY_PATH);
        return 0;
      }
      break;
#if (1) //---- 两点距离测试 ----//
    case 'A':
      {
        //lati_1 = gps.latitude_InM;
        //longti_1 = gps.longitude_InM;
        //P1 = [ 2410.087053 , 6973.595493 ] 
        // [ 2410.086907 6973.595328 47.811000 4 ] [ 313.588409 3 ] [ 11 ]  [ 658  657 ]- 01
        lati_1 = 2410.086907 ;
        longti_1 = 6973.595328;
        printf("\r\nP1 = [ %lf , %lf ] \r\n", lati_1, longti_1);  
      }
      break;
    case 'B':
      {
#if (TEST_CASE == DEMO_01_TEST)
        //double lati_2 = gps.latitude_InM;
        //double longti_2 = gps.longitude_InM;
        // P2 = [ 2410.096525  , 6973.602617 ] 
        // [ 2410.096257 6973.601846 48.063000 4 ] [ 59.537300 3 ] [ 13 ]  [ 4378  4377 ] - 02
        // [ 2410.103332 6973.599142 48.320000 4 ] [ 46.159100 3 ] [ 11 ]  [ 138  138 ]- 03
        double lati_2 = 2410.096257;
        double longti_2 = 6973.601846;
        
        double m = Get_TwoPointDistance(lati_1, longti_1, lati_2, longti_2);
        printf("\r\nP2 = [ %lf  , %lf ] \r\n", lati_2, longti_2);    
        printf("m = %lf \r\n\r\n", m);
        if(demo_01 == NULL)
        {
          demo_01 = DEMO_01_Init(lati_1, longti_1, lati_2, longti_2);
          if(demo_01) 
          {
            printf("Back To Origin, and insert [G] !\r\n");
          }
        }
#endif

#if (TEST_CASE == DEMO_02_TEST)
        // [ 2410.096257 6973.601846 48.063000 4 ] [ 59.537300 3 ] [ 13 ]  [ 4378  4377 ] - 02
        // [ 2410.103332 6973.599142 48.320000 4 ] [ 46.159100 3 ] [ 11 ]  [ 138  138 ]- 03
        double lati_2 = 2410.096257;
        double longti_2 = 6973.601846;
        double lati_3 = 2410.103332;
        double longti_3 = 6973.599142;
        double m = Get_TwoPointDistance(lati_1, longti_1, lati_2, longti_2);
        printf("\r\nP2 = [ %lf  , %lf ] \r\n", lati_2, longti_2);    
        printf("m = %lf \r\n\r\n", m);
        if(demo_02 == NULL)
        {
          double lati_m[3];
          double longti_m[3];
          int num_point = 3; //2
          lati_m[0] = lati_1; lati_m[1] = lati_2; lati_m[2] = lati_3;
          longti_m[0] = longti_1; longti_m[1] = longti_2; longti_m[2] = longti_3;
          demo_02 = DEMO_02_Init(lati_m, longti_m, num_point, log);
          if(demo_02) 
          {
            printf("Back To Origin, and insert [G] !\r\n");
          }
        }
#endif

#if (TEST_CASE == DEMO_03_TEST)
        double lati_2 = 2410.096257;
        double longti_2 = 6973.601846;
        double lati_3 = 2410.103332;
        double longti_3 = 6973.599142;
        double m = Get_TwoPointDistance(lati_1, longti_1, lati_2, longti_2);
        printf("\r\nP2 = [ %lf  , %lf ] \r\n", lati_2, longti_2);    
        printf("m = %lf \r\n\r\n", m);
        if(demo_03 == NULL)
        {
          double lati_m[3];
          double longti_m[3];
          int num_point = 3; //2
          lati_m[0] = lati_1; lati_m[1] = lati_2; lati_m[2] = lati_3;
          longti_m[0] = longti_1; longti_m[1] = longti_2; longti_m[2] = longti_3;
          demo_03 = DEMO_03_Init(lati_m, longti_m, num_point, log);
          if(demo_03) 
          {
            printf("Back To Origin, and insert [G] !\r\n");
          }
        }
#endif
      }
      break;
      case 'G':
      {
#if (TEST_CASE == DEMO_01_TEST)
        if((demo_01) && (pro != 2))
        {
          printf("\r\nGo To Destination \r\n");
          pro = 2;
        }
#endif

#if (TEST_CASE == DEMO_02_TEST)
        if((demo_02) && (pro != 4))
        {
          printf("\r\nGo To Destination \r\n");
          pro = 4;
        }
#endif

#if (TEST_CASE == DEMO_03_TEST)
        if((demo_03) && (pro != 6))
        {
          printf("\r\nGo To Destination \r\n");
          pro = 6;
        }
#endif
      }
      break;
      case 'C':
      {
        double lati_2 = gps.latitude_InM;
        double longti_2 = gps.longitude_InM;
        double m = Get_TwoPointDistance(lati_1, longti_1, lati_2, longti_2);
        printf("m = %lf \r\n", m);
      }
      break;
#endif
    default:
      {
        printf("-%c-\r\n", keyboard_char);
      } 
      break;  
    }

#if (1)
    switch(pro)
    {
      case 0: 
#if (TEST_CASE == CAR_AHEAD_TEST) 
        if(system_time_in_sec >= 2) 
        {
          gps.latitude_InM = 40.113966d * 60.0d;
          gps.longitude_InM = 116.394046d * 60.0d;
          double dst_la = 40.113996d * 60.0d;
          double dst_long = 116.394086d * 60.0d;
          gps.Yaw = 45;
          demo_01 = DEMO_01_Init(gps.latitude_InM, gps.longitude_InM, dst_la, dst_long); //gps.Yaw
          pro = 3;
        }
#endif

#if (SIMULATE_ENABLE) 
#if (TEST_CASE == DEMO_02_TEST) 
        if(system_time_in_sec >= 2) 
        {
          lati_1 = 2410.087053 ;
          longti_1 = 6973.595493;
          double lati_2 = 2410.096525;
          double longti_2 = 6973.602617;
          double m = Get_TwoPointDistance(lati_1, longti_1, lati_2, longti_2);
          printf("\r\nP2 = [ %lf  , %lf ] \r\n", lati_2, longti_2);    
          printf("m = %lf \r\n\r\n", m);
          if(demo_02 == NULL)
          {
            double lati_m[2];
            double longti_m[2];
            int num_point = 2;
            lati_m[0] = lati_1; lati_m[1] = lati_2;
            longti_m[0] = longti_1; longti_m[1] = longti_2;
            demo_02 = DEMO_02_Init(lati_m, longti_m, num_point, log);
          }
          pro = 5;
        }
#endif
#endif
        break;
      case 1:
#if (SIMULATE_ENABLE) 
#if (TEST_CASE == DEMO_01_TEST)
        {
          gps.latitude_InM = 40.113956d * 60.0d;
          gps.longitude_InM = 116.394046d * 60.0d;
          DEMO_01_Task(demo_01, &gps, fd_car);
        }
#endif
#endif
        break;
      case 2:
        {
#if (TEST_CASE == DEMO_01_TEST)
          int ret = DEMO_01_Task(demo_01, &gps, fd_car) ;
          if (ret < 0) 
          {
            printf("DEMO_01_Task Finish !\r\n");
            pro = 0xff;
          }
#endif
        }
        break;
      case  3:
        {
          CAR_GoAhead_Task(demo_01, fd_car);
        }
        break;
      case 4:
        {
#if (TEST_CASE == DEMO_02_TEST)
          int ret = DEMO_02_Task(demo_02, &gps, fd_car, log) ;
          if (ret < 0) 
          {
            printf("DEMO_02_Task Finish !\r\n");
            pro = 0xff;
          }
#endif
        }
      break;
      case 5:
#if (SIMULATE_ENABLE) 
#if (TEST_CASE == DEMO_02_TEST)
        {          
          gps.latitude_InM = lati_1 + 0.003;
          gps.longitude_InM = longti_1 + 0.003;
          gps.Yaw = 0.0;
          int ret = DEMO_02_Task(demo_02, &gps, fd_car, log);
          if (ret < 0) 
          {
            printf("DEMO_02_Task Finish !\r\n");
            pro = 0xff;
          }
        }
#endif
#endif
        break;
      case 6:
        {
#if (TEST_CASE == DEMO_03_TEST)
          int ret = DEMO_03_Task(demo_03, &gps, fd_car, log) ;
          if (ret < 0) 
          {
            printf("DEMO_03_Task Finish !\r\n");
            pro = 0xff;
          }
#endif
        }
      break;
      case 0xFF:
        break;
      default:;
    }
#endif
    usleep(1000);
  }

  DEMO_01_Release(demo_01);
  DEMO_02_Release(demo_02);
  DEMO_03_Release(demo_03);
  printf("\n---- test  end ----\n");
  UART0_Close(fd_gps);   
  UART0_Close(fd_car); 
  UART0_Close(fd_radio); 
  if(log != NULL) Log_Release(log);
  
	return 0;
}





#if (0)
    if(fd_car & 0x0)
    {
      static int pro = 0;
      switch(pro)
      {
        case 0:
        {
          if(system_time_in_sec >= 2)
          {
            SetMotoSpeed(fd_car, 100, 100);
            pro += 1;
          }
        }
        break;
        case 1:
        {
          if(system_time_in_sec >= 3)
          {
            SetMotoSpeed(fd_car, 200, 200);

            pro += 1;
          }
        }
        break;
        case 2:
        {
          if(system_time_in_sec >= 4)
          {
            SetMotoSpeed(fd_car, 300, 300);
            pro += 1;
          }
        }
        break;
        case 3:
        {
          if(system_time_in_sec >= 5)
          {
            SetMotoSpeed(fd_car, 400, 400);
            pro += 1;
          }
        }
        break;
        case 4:
        {
          if(system_time_in_sec >= 6)
          {
            SetMotoSpeed(fd_car, 300, 300);
            pro += 1;
          }
        }
        break;
        case 5:
        {
          if(system_time_in_sec >= 7)
          {
            SetMotoSpeed(fd_car, 200, 200);
            pro += 1;
          }
        }
        break;
        case 6:
        {
          if(system_time_in_sec >= 8)
          {
            SetMotoSpeed(fd_car, 100, 100);
            pro += 1;
          }
        }
        break;
        case 7:
        {
          if(system_time_in_sec >= 9)
          {
            SetMotoSpeed(fd_car, 0, 0);
            pro += 1;
          }
        }
        break;
        case 8:
        {
          if(system_time_in_sec >= 10)
          {
            loop_flag = 0;
            pro += 1;
          }
        }
        break;
      }
    }
#endif

#if (0)
  system(STTY_US TTY_PATH);
  timer_init();
  while(loop_flag)
  {
    static int sec_bk = 0;
    timer_check();
    keyboard_char = get_char();
    if(sec_bk != system_time_in_sec)
    {
      sec_bk = system_time_in_sec;
      printf("second = %d \r\n", sec_bk);
    }
    switch(keyboard_char)
    {
    case 0: break;
    case 3: //ctrl+c
      {
        system(STTY_DEF TTY_PATH);
        return 0;
      }
      break;
    default:
      {
        printf("-%c-\r\n", keyboard_char);
      } 
      break;  
    }
    usleep(1000);
  }
#endif









  
