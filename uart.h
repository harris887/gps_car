#ifndef _UART_H_
#define _UART_H_
#include "modbus.h"

//宏定义  
#define FALSE  -1  
#define TRUE   0  

#define TTY_PATH            "/dev/tty"
#define STTY_US             "stty raw -echo -F "
#define STTY_DEF            "stty -raw echo -F "

typedef struct
{
  unsigned char Flag;
  unsigned char Index;
  char BUF[256];
}RECORD_BUF;
#define NMEA_R_BUF_LEN 16

extern RECORD_BUF NMEA_R_BUF[NMEA_R_BUF_LEN];

int UART0_Open(int fd,char* port);
void UART0_Close(int fd)  ;
int UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity) ;
int UART0_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity);
int UART0_Recv(int fd, char *rcv_buf,int data_len,int timeout_ms)  ;
int UART0_Send(int fd, char *send_buf,int data_len) ;
int NMEA_RX_Task(int fd)  ;  


char get_char(void);
void SetMotoSpeed(int fd_car, int left, int right);
int MODBUS_UART_RX_Task(int fd);  






#endif

