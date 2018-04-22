           #ifndef _modbus_h_
#define _modbus_h_

typedef unsigned char  u8;
typedef signed char    s8;
typedef unsigned short u16;
typedef signed short   s16;
typedef unsigned int   u32;
typedef signed int     s32;


#define MAGIC_WORD  0x1A2B

#define return_OK                 0x00  // 一帧数据 OK 
#define illegal_function         0x01  // 功能码错误
#define illegal_register         0x02  // 寄存器地址错误
#define illegal_data              0x03  // 数据错误
#define crc_err                   0x04  //CRC 校验错误
#define switch_err                0x05  //写数据时开关没开
#define switch_value_err          0x06  //写开关值错误

#define CMD_ModBus_ReadEx       0x03
#define CMD_ModBus_Read         0x04
#define CMD_ModBus_Write        0x06
#define CMD_ModBus_WriteMore    0x16

typedef struct 
{
  u8  machine_state;
  u8  ModBus_CMD;
  u8  Probe_Slave_Addr;
  u16 Read_Register_Addr;
  u16 Write_Register_Addr;
  u16 Read_Register_Num;
  u16 Read_index;
  u8  Write_Register_Data[120];
  u16 Write_Register_Num;
  u16 Write_Register_Data_One;
  u8  Probe_state;
  u8  err_state;//返回错误代码。
  

  u8  data[256];
  u8  data_index;
  u8  write_one_receive_timer;
  u8  write_more_receive_timer;
  u8  read_receive_timer;
}MODBUS;


typedef struct
{
  //-- 0x0 ~ 0x1F--//
  u16 rsv_000;
  u16 SLAVE_ADDR;
  u16 COMM_BD;
  u16 rsv_001[29];

  //-- 0x20 ~ 0x3F--//
  u16 VEHICLE_CONTROL;
  u16 rsv_002[7];
  u16 ORIGIN_LONGTI[2];
  u16 ORIGIN_LATI[2];
  u16 rsv_003[20];

  //-- 0x40 ~ 0x4F--//
  u16 VEHICLE_STATUS;
  u16 BATT_VOLT;
  u16 rsv_004[6];
  u16 MOTO_CURRENT[4];
  u16 rsv_005[4];

  //-- 0x50 ~ 0x1FF--//
  s16 VEHICLE_LOCATION_X;
  s16 VEHICLE_LOCATION_Y;
  s16 VEHICLE_LOCATION_YAW;
  s16 VEHICLE_SPEED;
  u16 VEHICLE_LONGTI[2];
  u16 VEHICLE_LATI[2];
  u16 VEHICLE_YAW[2];
  u16 GPS_LOCATION_QUALITY;
  u16 GPS_YAW_QUALITY;
  u16 rsv_006[420];

  //-- 0x200 ~ 0x21F--//
  u16 MAP_NUM;
  u16 MAP_INDEX;
  u16 rsv_007[30];

  //-- 0x220 ~ 0x3FF--//
  u16 MAP_INFOR[16][2];
  u16 rsv_008[448];

  u16 MOD_REG_MAGIC_WORD;
}MOD_BUS_REG;

#define MOD_BUS_REG_NUM ((sizeof(MOD_BUS_REG) >> 1) - 1)

extern MODBUS MODBUS_Radio;
extern MOD_BUS_REG MOD_BUS_Reg;

extern void Analysis_Receive_From_Master(u8 data, MODBUS* A8_Modbus, MOD_BUS_REG* MOD_BUS_Reg, int fd_radio);
extern u16 ModBus_CRC16_Calculate(u8 *aStr , u8 alen);
extern int LoadModbusReg(void);

extern int Radio_Send(int fd, char *send_buf, int data_len, int delay_ms); 
extern void Radio_Send_Task(void);
#endif

