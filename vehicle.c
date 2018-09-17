#include <stdio.h>      
#include <stdlib.h>   
#include <unistd.h>     /*Unix 标准函数定义*/  
#include <sys/types.h>   
#include <sys/stat.h>     
#include <fcntl.h>      /*文件控制定义*/  
#include <termios.h>    /*PPSIX 终端控制定义*/  
#include <errno.h>      /*错误号定义*/  
#include <string.h>  
#include "stdbool.h"
#include "uart.h"
#include "misc.h"
#include "vehicle.h"

#define VEHICLE_COMM_RX_TIMEOUT 5 
#define VEHICLE_COMM_CYCLE_MS   10
#define VEHICLE_READ_INFOR_MS   1000
#define VEHICLE_READ_DIDO_MASK  0x1
#define VEHICLE_READ_BMS_MASK   0x2

MODBUS_SAMPLE MODBUS_Vehicle = {
  .MachineState = 0,
  .read_success_num = 0,
  .write_success_num = 0,
};

DIDO_INFOR DIDO_Infor = 
{
  .DAM0808_Input = 0,
  .DAM0808_Output = 0,
  .DAM0404_Input = {0, 0, 0, 0},
  .DAM0404_Output = 0,
  .ReadNum = 0,
};

BMS_INFOR BMS_Infor = 
{
  .Valid = 0,
  .Num = 0,
};

int VEHICLE_READ_INFOR_Flag = 0;
int VEHICLE_SET_SPEED_Flag = 0;
int VEHICLE_SPEED_left = 0;
int VEHICLE_SPEED_right = 0;
int VEHICLE_RX_ErrorNum = 0;

void Analysis_Receive_From_Vehicle(u8 data, MODBUS_SAMPLE* pMODBUS);
void SetMotoSpeed(int fd_car, int left, int right);
void GetDidoInfor(int fd_car);
void GetBmsInfor(int fd_car);
u16 Get_BD_U16(u8** beam);
u32 Get_BD_U32(u8** beam);  
//---- 串口发送/接收 ----//
int VEHICLE_UART_TRANS_Task(int fd)  
{  
  static s64 time_bk_0 = 0;
  static s64 time_bk_1 = 0;
  static s64 time_bk_2 = 0;
  static s64 last_rx_timestamp = 0;
  s64 time;
  int i;
  if(fd == 0) return -1;  

  time = GetCurrentTimeMs();
  if(time_bk_0 != time);
  {
    char temp_buf[128];
    int temp_len;
    time_bk_0 = time;
    temp_len = read(fd, temp_buf, 128); 
    if(temp_len > 0)
    {
      last_rx_timestamp = time;
      for(i = 0; i < temp_len; i++)
      {
        Analysis_Receive_From_Vehicle(temp_buf[i], &MODBUS_Vehicle);
      }
    }
    //---- 接收超时处理 ----//
    if(time >= (last_rx_timestamp + VEHICLE_COMM_RX_TIMEOUT)) 
    {
      if(MODBUS_Vehicle.MachineState != 0)
      {
        MODBUS_Vehicle.MachineState = 0;
        VEHICLE_RX_ErrorNum += 1;
        //printf("Reset MODBUS_Vehicle.MachineState once !\r\n");
      }
    }
  }
  


  if((time - time_bk_1) > VEHICLE_COMM_CYCLE_MS)
  {
    time_bk_1 = time;
    
    if(VEHICLE_SET_SPEED_Flag)
    {
      SetMotoSpeed(fd, VEHICLE_SPEED_left, VEHICLE_SPEED_right);
      VEHICLE_SET_SPEED_Flag = 0;
    }
    else if(VEHICLE_READ_INFOR_Flag & VEHICLE_READ_DIDO_MASK)
    {
      GetDidoInfor(fd);
      VEHICLE_READ_INFOR_Flag &= (~VEHICLE_READ_DIDO_MASK);
    }
    else if(VEHICLE_READ_INFOR_Flag & VEHICLE_READ_BMS_MASK)
    {
      GetBmsInfor(fd);
      VEHICLE_READ_INFOR_Flag &= (~VEHICLE_READ_BMS_MASK);
    }
  }

  // Check [read vehicle infor] necessary
  if((time - time_bk_2) > VEHICLE_READ_INFOR_MS)
  {
    time_bk_2 = time;
    VEHICLE_READ_INFOR_Flag = (VEHICLE_READ_DIDO_MASK | VEHICLE_READ_BMS_MASK);
  }

	return 0; 
}

/*******************************************************************
函数名称:
函数功能:
*******************************************************************/
void Analysis_Receive_From_Vehicle(u8 data, MODBUS_SAMPLE* pMODBUS)
{
    switch(pMODBUS->MachineState)//初始化 默认 为 00;
    {
    case 0x00:
      {
        if(data >= MODE_BUS_VEHICLE_Addr)//从机地址
        {
          pMODBUS->MachineState = 0x01;
          pMODBUS->BufIndex = 0;
          pMODBUS->DataBuf[pMODBUS->BufIndex++] = data;
        }
        else
        {
          pMODBUS->MachineState = 0x0B;//缓冲数据区域清零要处理，中间数据为01，误认为是要从机地址。
          pMODBUS->BufIndex = 0;
        }  
      }
      break;
    case 0x01:
      {
        pMODBUS->DataBuf[pMODBUS->BufIndex++] = data;
        if(data == CMD_ModBus_Read) 
        {
          pMODBUS->MachineState = 0x02; 
          pMODBUS->ModBus_CMD = data;
          pMODBUS->read_receive_timer = 0;
        }
        else
        { 
          pMODBUS->MachineState = 0x0B;
          pMODBUS->BufIndex = 0;
        }
      }
      break;
      case 0x02: //read part 00
      {    
        //接收到读功能的字节数
        pMODBUS->DataBuf[pMODBUS->BufIndex++] = data;
        pMODBUS->read_receive_timer++;
        if(pMODBUS->read_receive_timer == 2 )
        {
          pMODBUS->Read_Register_Num = pMODBUS->DataBuf[pMODBUS->BufIndex-2]*256 + pMODBUS->DataBuf[pMODBUS->BufIndex-1];
          if(pMODBUS->Read_Register_Num <= 96) 
          {
            pMODBUS->MachineState = 0x03;
          }
          else
          {
            pMODBUS->MachineState = 0x00;
          }
          pMODBUS->read_receive_timer = 0;
        } 
      }
      break;
      case 0x03: //read part 01
      {   
        pMODBUS->DataBuf[pMODBUS->BufIndex++] = data;
        pMODBUS->read_receive_timer++;
        if(pMODBUS->read_receive_timer >= (pMODBUS->Read_Register_Num + 2))
        {
          u16 cal_crc;
          cal_crc=ModBus_CRC16_Calculate(pMODBUS->DataBuf,pMODBUS->Read_Register_Num+4);
              
          pMODBUS->receive_CRC_L = pMODBUS->DataBuf[pMODBUS->BufIndex-2];
          pMODBUS->receive_CRC_H = pMODBUS->DataBuf[pMODBUS->BufIndex-1];
          if(((cal_crc>>8) == pMODBUS->receive_CRC_H) && ((cal_crc&0xFF) == pMODBUS->receive_CRC_L))
          {
            pMODBUS->err_state = 0x00;//CRC校验正确 
            pMODBUS->read_success_num += 1;
            
            //正确读到数据
            if(pMODBUS->Read_Register_Num == 14)
            {
              u8* info = pMODBUS->DataBuf + 4, i;
 
              DIDO_Infor.DAM0808_Input = (info[0] << 8) | info[1];
              DIDO_Infor.DAM0808_Output = (info[2] << 8) | info[3];
              for(i = 0; i < 4; i++) DIDO_Infor.DAM0404_Input[i] = (info[4 + 2 * i] << 8) | info[5 + 2 * i];
              DIDO_Infor.DAM0404_Output = (info[12] << 8) | info[13];
              DIDO_Infor.ReadNum += 1;
              
              // -- To Radio Server --//
              for(i = 0; i < 4; i++) MOD_BUS_Reg.MOTO_CURRENT[i] = DIDO_Infor.DAM0404_Input[i];
              MOD_BUS_Reg.DIDO_DI8 = DIDO_Infor.DAM0808_Input;
              MOD_BUS_Reg.DIDO_DO12 = DIDO_Infor.DAM0808_Output | ((DIDO_Infor.DAM0404_Output & 0xF) << 8);

            }
            else if(pMODBUS->Read_Register_Num == 32)
            {
              u8 *bs = pMODBUS->DataBuf + 4, i;
              BMS_INFOR* bms = &BMS_Infor;

              bms->BAT_MV = Get_BD_U32(&bs);
              bms->BAT_MA = Get_BD_U32(&bs);
              for(i = 0; i < 3; i++)  bms->BAT_TEMP[i] = Get_BD_U16(&bs);
              bms->FCC = Get_BD_U32(&bs);;
              bms->RC = Get_BD_U32(&bs);;
              bms->RSOC = Get_BD_U16(&bs);
              bms->CycleCount = Get_BD_U16(&bs);
              bms->PackStatus = Get_BD_U16(&bs);
              bms->BatStatus = Get_BD_U16(&bs);
              bms->Valid = Get_BD_U16(&bs);
              bms->Num += 1;

              // -- To Radio Server --//
              u16* dst = MOD_BUS_Reg.BMS_MV;
              u8* src = pMODBUS->DataBuf + 4;
              for(i = 0; i < 15; i++) *dst++ = Get_BD_U16(&src); // 15 * HalfWord
            }
            else
            {

            }
          }    
          else	  
          {
             pMODBUS->err_state = 0x04;
          }   
          pMODBUS->BufIndex = 0;  
          pMODBUS->read_receive_timer = 0;  
          pMODBUS->MachineState = 0x00;                
        }  
      }
      break;
      case 0xb:
      {
      }
      break;      
      default:
      {
        pMODBUS->MachineState=0;
      }
    }
}

//--
const char CMD_SET_MOTO_SPEED[] = 
{0x01 ,0x16 ,0x00 ,0x38 ,0x00 ,0x08 ,
 0x00 ,0x00 ,0x00 ,0x64 ,//左前轮
 0x00 ,0x00 ,0x00 ,0x00 ,//右前轮
 0x00 ,0x00 ,0x00 ,0x00 ,//左后轮
 0x00 ,0x00 ,0x00 ,0x00 ,//右后轮
 0x5A ,0xB3 };

const char CMD_READ_DIDO[] = 
{0x01 ,0x04 ,0x00 ,0xB0 ,0x00 ,0x07 ,0xB0 ,0x2F};

const char CMD_READ_BMS[] = 
{0x01 ,0x04 ,0x00 ,0xC0 ,0x00 ,0x10 ,0xF1 ,0xFA};

void GetDidoInfor(int fd_car)
{
  if(fd_car)
  {
    UART0_Send(fd_car, (char*)CMD_READ_DIDO, sizeof(CMD_READ_DIDO));
  }
}

void GetBmsInfor(int fd_car)
{
  if(fd_car)
  {
    UART0_Send(fd_car, (char*)CMD_READ_BMS, sizeof(CMD_READ_BMS));
  }
}

void SetMotoSpeed(int fd_car, int left, int right)
{
  char cmd_buf[256];
  char offset = 8;
  unsigned short cal_crc;
  memcpy(cmd_buf, CMD_SET_MOTO_SPEED, sizeof(CMD_SET_MOTO_SPEED));
  if(left < 0) 
  {  
    cmd_buf[7] = 1;
    cmd_buf[7+offset] = 1;
    left = -left;
  }
  if(right < 0) 
  {  
    cmd_buf[11] = 1;
    cmd_buf[11+offset] = 1;
    right = -right;
  }
  cmd_buf[8] = (left >> 8) & 0xFF;
  cmd_buf[9] = (left >> 0) & 0xFF;
  cmd_buf[8+offset] = (left >> 8) & 0xFF;
  cmd_buf[9+offset] = (left >> 0) & 0xFF;

  cmd_buf[12] = (right >> 8) & 0xFF;
  cmd_buf[13] = (right >> 0) & 0xFF;
  cmd_buf[12+offset] = (right >> 8) & 0xFF;
  cmd_buf[13+offset] = (right >> 0) & 0xFF;

  cal_crc = ModBus_CRC16_Calculate((u8*)cmd_buf , sizeof(CMD_SET_MOTO_SPEED) - 2);
  cmd_buf[22] = cal_crc&0xFF;
  cmd_buf[23] = cal_crc>>8;   

  if(fd_car)
  {
    UART0_Send(fd_car, cmd_buf, sizeof(CMD_SET_MOTO_SPEED));
  }
}

void SetMotoSpeedAsync(int left, int right)
{
  VEHICLE_SPEED_left = left;
  VEHICLE_SPEED_right = right;
  VEHICLE_SET_SPEED_Flag = 1;
}

u16 Get_BD_U16(u8** beam) 
{
  u16 temp;
  u8* c = *beam;
  temp = ((u16)c[0] << 8) | ((u16)c[1] << 0);
  *beam = c + 2;
  return temp;
}

u32 Get_BD_U32(u8** beam) 
{
  u32 temp;
  u8* c = *beam;
  temp = ((u32)c[0] << 24) | ((u32)c[1] << 16) | ((u32)c[2] << 8) | ((u32)c[3] << 0);
  *beam = c + 4;
  return temp;
}

void PrintBmsInfor(void)
{
  printf("\r\n---------------------------\r\n");
  printf("----------- BMS -----------\r\n");
  printf("BAT_MV    : %d\r\n", BMS_Infor.BAT_MV);
  printf("BAT_MA    : %d\r\n", BMS_Infor.BAT_MA);
  printf("BAT_TEMP  : %d\r\n", BMS_Infor.BAT_TEMP[0]);
  printf("BAT_TEMP  : %d\r\n", BMS_Infor.BAT_TEMP[1]);
  printf("BAT_TEMP  : %d\r\n", BMS_Infor.BAT_TEMP[2]);
  printf("FCC       : %d\r\n", BMS_Infor.FCC);
  printf("RC        : %d\r\n", BMS_Infor.RC);  
  printf("RSOC      : %d\r\n", BMS_Infor.RSOC); 

  printf("CycleCount: %d  \r\n", BMS_Infor.CycleCount);
  printf("PackStatus: %04X\r\n", BMS_Infor.PackStatus);  
  printf("BatStatus : %04X\r\n", BMS_Infor.BatStatus); 
  printf("Valid     : %d  \r\n", BMS_Infor.Valid);  
  printf("Num       : %d  \r\n", BMS_Infor.Num); 
  printf("---------------------------\r\n");
}

void PrintDidoInfor(void)
{
  printf("\r\n------------------------------\r\n");
  printf("------------ DIDO ------------\r\n");
  printf("DAM0808_Input    : %04X\r\n", DIDO_Infor.DAM0808_Input);
  printf("DAM0808_Output   : %04X\r\n", DIDO_Infor.DAM0808_Output);
  printf("DAM0404_Input_0  : %04X\r\n", DIDO_Infor.DAM0404_Input[0]);
  printf("DAM0404_Input_1  : %04X\r\n", DIDO_Infor.DAM0404_Input[1]);
  printf("DAM0404_Input_2  : %04X\r\n", DIDO_Infor.DAM0404_Input[2]);
  printf("DAM0404_Input_3  : %04X\r\n", DIDO_Infor.DAM0404_Input[3]);
  printf("DAM0404_Output   : %04X\r\n", DIDO_Infor.DAM0404_Output);  
  printf("ReadNum          : %d  \r\n", DIDO_Infor.ReadNum); 
  printf("------------------------------\r\n");
}

