#ifndef _vehicle_h_
#define _vehicle_h_
#include "type_def.h"

typedef struct
{
  u16 DAM0808_Input;
  u16 DAM0808_Output;
  u16 DAM0404_Input[4];
  u16 DAM0404_Output;
  u32 ReadNum;
}DIDO_INFOR;

typedef struct
{
  u32 BAT_MV;
  u32 BAT_MA;
  u16 BAT_TEMP[3];
  u32 FCC;
  u32 RC;
  u16 RSOC;
  u16 CycleCount;
  u16 PackStatus;
  u16 BatStatus;
  u16 Valid;
  u32 Num;
}BMS_INFOR;

typedef struct 
{
  u8 MachineState;
  u8 BufIndex;
  u8 read_receive_timer;
  u8 receive_CRC_H;
  u8 receive_CRC_L;
  u8 rev0;
  
  u8 ModBus_CMD;
  u8 err_state;
  u32 read_success_num;
  u32 write_success_num;
  u16 Read_Register_Num;
  u8 DataBuf[256];
}MODBUS_SAMPLE;

#define MODE_BUS_VEHICLE_Addr  1

extern MODBUS_SAMPLE MODBUS_Vehicle;
extern DIDO_INFOR DIDO_Infor;
extern BMS_INFOR BMS_Infor;
extern int VEHICLE_RX_ErrorNum;

void SetMotoSpeedAsync(int left, int right);
int VEHICLE_UART_TRANS_Task(int fd);
void PrintBmsInfor(void);
void PrintDidoInfor(void);

#endif

