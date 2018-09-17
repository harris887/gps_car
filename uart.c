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
#include "misc.h"
#include "uart.h"

//---- debug data ----// 
int debug_rx_bytes = 0;

//-------------------------------------------------------// 
int NMEA_R_BUF_Index=0;
RECORD_BUF NMEA_R_BUF[NMEA_R_BUF_LEN] = {
  {0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},
  {0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0} };
//-------------------------------------------------------//   

void SetMotoSpeed(int fd_car, int left, int right);
//-------------------------------------------------------//


#define BLOCK_STOP_MODE						0
#define DEFAULT_RX_TIMEOUT_IN_MS	100

#define RX_FRAME_TIMEOUT_IN_MS    5
#define DEFAULT_UART_BD           115200

/******************************************************************* 
* 名称：		UART0_Open 
* 功能：  	打开串口并返回串口设备文件描述 
* 入口参数：fd    :文件描述符     port :串口号(ttyS0,ttyS1,ttyS2) 
* 出口参数：正确返回为1，错误返回为0 
*******************************************************************/  
int UART0_Open(int fd,char* port)  
{  
  #if (BLOCK_STOP_MODE)   
	fd = open( port, O_RDWR|O_NOCTTY|O_NDELAY);  //
	if (FALSE == fd)  
	{  
		perror("Can't Open Serial Port");  
		return(FALSE);  
	}  
	
	//恢复串口为阻塞状态                                 
	if(fcntl(fd, F_SETFL, 0) < 0)  
	{  
		printf("fcntl failed!\n");  
		return FALSE;  
	}       
	else  
	{  
		printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));  
 	}
 	#else
	fd = open( port, O_RDWR|O_NOCTTY|O_NDELAY|O_NONBLOCK);  //
	if (FALSE == fd)  
	{  
		perror("Can't Open Serial Port");  
		return(FALSE);  
	}  
 	#endif
 	  
	//测试是否为终端设备      
	//if(0 == isatty(STDIN_FILENO))  
	if(0)
        {  
		printf("standard input is not a terminal device\n");  
		return FALSE;  
	}  
  else  
	{  
		printf("isatty success!\n");  
	}                
	printf("fd->open=%d\n",fd);  
	return fd;  
}  

/******************************************************************* 
* 名称：		UART0_Close 
* 功能：		关闭串口并返回串口设备文件描述 
* 入口参数：fd    :文件描述符     port :串口号(ttyS0,ttyS1,ttyS2) 
* 出口参数：void 
*******************************************************************/  
void UART0_Close(int fd)  
{  
    close(fd);  
}  
   
/******************************************************************* 
* 名称：		UART0_Set 
* 功能：		设置串口数据位，停止位和效验位 
* 入口参数：fd 串口文件描述符 
* 					speed     串口速度 
* 					flow_ctrl   数据流控制 
*						databits   数据位   取值为 7 或者8 
*						stopbits   停止位   取值为 1 或者2 
*						parity     效验类型 取值为N,E,O,,S 
*出口参数： 正确返回为1，错误返回为0 
*******************************************************************/  
int UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)  
{  
	int   i , bd_valid = 0;  
	int   status;  
	int   speed_arr[] = { B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300};  
	int   name_arr[] = {115200,  57600,  38400,  19200,  9600,  4800,  2400,  1200,  300};  

	struct termios options;  
     
	/*tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1. 
	 */  
	if(tcgetattr(fd, &options) != 0)  
	{  
		printf("Setup SerialPort\n");      
		return FALSE ;   
	}  
    
	//设置串口输入波特率和输出波特率  
	for (i= 0; i < sizeof(speed_arr) / sizeof(int);  i++)  
	{  
		if  (speed == name_arr[i])  
		{               
			cfsetispeed(&options, speed_arr[i]);   
			cfsetospeed(&options, speed_arr[i]);  
			printf("Band Rate = %d\n", speed); 
      bd_valid = 1; 
      break;
		}  
	}
  if(bd_valid == 0)
  {
    printf("Unsupported data size\n");     
  }    
     
	//修改控制模式，保证程序不会占用串口  
	options.c_cflag |= CLOCAL;  
	
	//修改控制模式，使得能够从串口中读取输入数据  
	options.c_cflag |= CREAD;  
    
	//设置数据流控制  
	switch(flow_ctrl)  
	{  
	case 0 ://不使用流控制  
		options.c_cflag &= ~CRTSCTS;  
		break;     
	case 1 ://使用硬件流控制  
		options.c_cflag |= CRTSCTS;  
		break;  
	case 2 ://使用软件流控制  
		options.c_cflag |= IXON | IXOFF | IXANY;  
		break;  
	}  
	
	//设置数据位  
	//屏蔽其他标志位  
	options.c_cflag &= ~CSIZE;  
	switch (databits)  
	{    
		case 5    :  
			options.c_cflag |= CS5;  
			break;  
		case 6    :  
			options.c_cflag |= CS6;  
			break;  
		case 7    :      
			options.c_cflag |= CS7;  
			break;  
		case 8:      
			options.c_cflag |= CS8;  
			break;    
		default:     
			printf("Unsupported data size\n");  
			return FALSE;   
	}  
	
	//设置校验位  
	switch (parity)  
	{    
		case 'n':  
		case 'N': //无奇偶校验位。  
			options.c_cflag &= ~PARENB;  
      //options.c_iflag |= INPCK;  
			options.c_iflag &= ~(BRKINT|ICRNL|INPCK|ISTRIP|IXON);   //harris 20171005,  cannot rx 0x11   
			break;   
		case 'o':    
		case 'O'://设置为奇校验      
			options.c_cflag |= (PARODD | PARENB);   
      //options.c_iflag |= INPCK;  
			options.c_iflag &= ~(BRKINT|ICRNL|INPCK|ISTRIP|IXON);               
			break;   
		case 'e':   
		case 'E'://设置为偶校验    
			options.c_cflag |= PARENB;         
			options.c_cflag &= ~PARODD;         
      //options.c_iflag |= INPCK;  
			options.c_iflag &= ~(BRKINT|ICRNL|INPCK|ISTRIP|IXON);       
			break;  
		case 's':  
		case 'S': //设置为空格   
			options.c_cflag &= ~PARENB;  
      //options.c_iflag |= INPCK;  
			options.c_iflag &= ~(BRKINT|ICRNL|INPCK|ISTRIP|IXON);  
			break;   
		default:    
			fprintf(stderr,"Unsupported parity\n");      
			return FALSE;   
	}   
    
	// 设置停止位   
	switch (stopbits)  
	{    
		case 1:     
			options.c_cflag &= ~CSTOPB; 
			break;   
		case 2:     
			options.c_cflag |= CSTOPB; 
			break;  
		default:     
			printf("Unsupported stop bits\n");   
			return FALSE;  
	}  
     
	//修改输出模式，原始数据输出  
  options.c_oflag &= ~OPOST;  
    
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);//我加的  
	//options.c_lflag &= ~(ISIG | ICANON);  
     
	//设置等待时间和最小接收字符  
	options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */    
	options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */  
     
	//如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读  
	tcflush(fd,TCIFLUSH);  
     
  //激活配置 (将修改后的termios数据设置到串口中）  
	if (tcsetattr(fd,TCSANOW,&options) != 0)    
	{  
		printf("com set error!\n");    
		return FALSE;   
	}  
	return TRUE;   
}  
/******************************************************************* 
* 名称：		UART0_Init() 
* 功能：  	串口初始化 
* 入口参数：fd       :  文件描述符    
*           speed  :  串口速度 
*           flow_ctrl  数据流控制 
*           databits   数据位   取值为 7 或者8 
*           stopbits   停止位   取值为 1 或者2 
*           parity     效验类型 取值为N,E,O,,S 
*                       
* 出口参数：正确返回为1，错误返回为0 
*******************************************************************/  
int UART0_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity)  
{  
	int err;  
	//设置串口数据帧格式  
	if(UART0_Set(fd, speed, flow_ctrl, databits, stopbits, parity) == FALSE)  
	{                                                           
		return FALSE;  
	}  
	return  TRUE;   
}  
   
/******************************************************************* 
* 名称：		UART0_Recv 
* 功能：		接收串口数据 
* 入口参数：fd          :文件描述符     
*           rcv_buf     :接收串口中数据存入rcv_buf缓冲区中 
*           data_len    :一帧数据的长度 
*						timeout_ms  :超时时间
* 出口参数：正确返回为1，错误返回为0 
*******************************************************************/  
int UART0_Recv(int fd, char *rcv_buf,int data_len,int timeout_ms)  
{  
	#if (!BLOCK_STOP_MODE)
	//int timeout_ms = 10;
	int tmp;
	int len = 0;
	do{
		usleep(1000);
		tmp = read(fd,rcv_buf+len,data_len-len); 
		//printf("tmp = %d\n", tmp);
		if(tmp > 0) len += tmp;
		timeout_ms -= 1;
	}while((len<data_len)&&(timeout_ms>0));
	printf("timeout_ms_left = %d\n", timeout_ms);
	
	return len; 
	#else
	int len,fs_sel;  
	fd_set fs_read;       
	struct timeval time;  
     
	FD_ZERO(&fs_read);  
	FD_SET(fd,&fs_read);  
     
	time.tv_sec = 0;  
	time.tv_usec = timeout_ms * 1000;  
     
	//使用select实现串口的多路通信  
	fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);  
	if(fs_sel)  
	{  
		len = read(fd,rcv_buf,data_len);  
		//printf("I am right!(version1.2) len = %d fs_sel = %d\n",len,fs_sel);  
		return len;  
	}  
	else  
	{  
		//printf("Sorry,I am wrong!");  
		return FALSE;  
	}       
	#endif
}  
/******************************************************************** 
* 名称： 		UART0_Send 
* 功能：  	发送数据 
* 入口参数：fd          :文件描述符     
*						send_buf    :存放串口发送数据 
*						data_len    :一帧数据的个数 
* 出口参数：正确返回为1，错误返回为0 
*******************************************************************/  
int UART0_Send(int fd, char *send_buf,int data_len)  
{  
	int len = 0;  
     
	len = write(fd,send_buf,data_len);  
	if (len == data_len )  
	{  
		return len;  
	}       
	else     
	{       
		//tcflush(fd,TCOFLUSH);  
		return FALSE;  
	}
  tcflush(fd,TCOFLUSH); 
}  

//---- 串口接收任务 ----//
int NMEA_RX_Task(int fd)  
{  
#if (!BLOCK_STOP_MODE)
  static s64 time_bk = 0;
  static s64 last_rx_timestamp = 0;
  static unsigned char ack_buf[256];
  static unsigned char ack_bytes;
  s64 time;
  if(fd == 0) return -1;  

  time = GetCurrentTimeMs();
  if(time_bk != time);
  {
    char temp_buf[32], i, temp, index0, new_index;
    int temp_len;
    time_bk = time;
    temp_len = read(fd, temp_buf, 32); 
    
    if(temp_len > 0)
    {
      debug_rx_bytes += temp_len;
      for(i = 0; i < temp_len; i++)
      {
        temp = temp_buf[i];
        if(temp=='$')//new Frame 
        {
          if(NMEA_R_BUF_Index>=(NMEA_R_BUF_LEN-1)) new_index=0;
          else new_index=NMEA_R_BUF_Index+1;
          if(NMEA_R_BUF[new_index].Flag==0)
          {
            NMEA_R_BUF[NMEA_R_BUF_Index].Flag=2;
            NMEA_R_BUF_Index=new_index;
            NMEA_R_BUF[NMEA_R_BUF_Index].Flag=1;
            NMEA_R_BUF[NMEA_R_BUF_Index].Index=0;
          }
          else
          {
            NMEA_R_BUF[NMEA_R_BUF_Index].Flag=1;
            NMEA_R_BUF[NMEA_R_BUF_Index].Index=0;
          }
        }
        index0=NMEA_R_BUF[NMEA_R_BUF_Index].Index;
        NMEA_R_BUF[NMEA_R_BUF_Index].BUF[index0]=temp;
        NMEA_R_BUF[NMEA_R_BUF_Index].Index++;
      }
    }
    //---- 接收超时处理 ----//
    if(time >= (last_rx_timestamp + RX_FRAME_TIMEOUT_IN_MS))
    {
      
    }

    //---- parse NMEA ----//
    for(i=0;i<NMEA_R_BUF_LEN;i++)
    {
      if(NMEA_R_BUF[i].Flag==2)
      {
        NMEA_parse(NMEA_R_BUF[i].BUF);
        NMEA_R_BUF[i].Flag=0;
      }
    }
  }
  #endif
	return 0; 
}



char get_char(void)
{
    fd_set rfds;
    struct timeval tv;
    char ch = 0;

    FD_ZERO(&rfds);
    FD_SET(0, &rfds);
    tv.tv_sec = 0;
    tv.tv_usec = 10; //设置等待超时时间

    //检测键盘是否有输入
    if (select(1, &rfds, NULL, NULL, &tv) > 0)
    {
        ch = getchar(); 
    }

    return ch;
}


