#include "modbus.h"
#include "uart.h"
#include "string.h"
#include "stdio.h"
#include "misc.h"
#include "module_core.h"

#define RADIO_ACK_DELAY_MS	 50 //60
#define SAVE_ONCE_BETWEEN_MS 60000  //1 minute
MOD_BUS_REG MOD_BUS_Reg;
MOD_BUS_REG MOD_BUS_Reg_Backup;

/*默认值*/
const MOD_BUS_REG DEFAULT_MOD_BUS_Reg=
{
  .rsv_000 = 0,
  .SLAVE_ADDR = 1,
  .COMM_BD = 4,

  //-- 0x20 ~ 0x3F--//
  .VEHICLE_CONTROL = 0, 
  .VEHICLE_TEST_CONTROL = 0,
  .ORIGIN_LONGTI = {0x3f80, 0x0000},//1.0f
  .ORIGIN_LATI = {0x3faa,0xaa8f},   //1.3333f

  //-- 0x40 ~ 0x4F--//
  .VEHICLE_STATUS = 0x0F0F, //
  .BATT_VOLT = 480,
  .MOTO_CURRENT = {0, 10, 20, 30},

  //-- 0x50 ~ 0x1FF--//
  .VEHICLE_LOCATION_X = 0,
  .VEHICLE_LOCATION_Y = 1,
  .VEHICLE_LOCATION_YAW = 180,
  .VEHICLE_SPEED = 0,
  .VEHICLE_LONGTI = {0x4006, 0x6666}, //2.1f
  .VEHICLE_LATI = {0x404c, 0xcccd},   //3.2f
  .VEHICLE_YAW = {0x4270, 0x0000},    //60.0
  .GPS_LOCATION_QUALITY = 4,
  .GPS_YAW_QUALITY = 3,

  //-- 0x200 ~ 0x21F--//
  .MAP_NUM = 0,
  .MAP_INDEX = 0,

  .MOD_REG_MAGIC_WORD = MAGIC_WORD,
};

MODBUS MODBUS_Radio = {0};
u8 MOD_BUS_REG_FreshFlag = 0;

u8 AckModBusCrcError(u8 CMD_ModBus, int fd_radio);
u8 AckModBusFunctionError(u8 CMD_ModBus, int fd_radio);
u8 AckModBusReadReg(u16 reg_addr, u16 reg_num, int fd_radio);
u8 AckModBusWriteOneReg(u16 reg_addr,u16 reg_value, int fd_radio);
u8 AckModBusWriteMultiReg(u16 reg_addr, u16 reg_num, u8* pData, int fd_radio);

//--------------------------------------//
typedef struct
{ 
  int fd;
  u8 data[256];
  u8 data_len;
  s64 delay_ms;
  s64 timestamp;
}RADIO_INFOR_CONTENT;  

#define RADIO_INFOR_CONTENT_LIST_NUM	 16
#define RADIO_INFOR_CONTENT_LIST_NUM_MASK (RADIO_INFOR_CONTENT_LIST_NUM - 1) 
RADIO_INFOR_CONTENT RADIO_INFOR_CONTENT_List[RADIO_INFOR_CONTENT_LIST_NUM]; 
int RADIO_INFOR_CONTENT_LIST_InIndex = 0;
int RADIO_INFOR_CONTENT_LIST_OutIndex = 0;
//--------------------------------------//

s64 SaveModbusReg_ms;
int LoadModbusReg(void)
{
  int size = 0, set_to_default = 0;
  FILE* modbus_bin = fopen("modbus_reg.bin", "rb");
  if(modbus_bin != NULL)
  {
    //printf("modbus_reg.bin exist!\r\n");
    fseek(modbus_bin, 0L, SEEK_END); 
    size = ftell(modbus_bin); 
    fseek(modbus_bin, 0L, SEEK_SET);
  }
  else
  {
    printf("modbus_reg.bin not exist!\r\n");
  }
  if(size != sizeof(MOD_BUS_REG))
  {
    if(modbus_bin) fclose(modbus_bin);
    set_to_default = 1;
    printf("modbus_bin size error ! %d\r\n", size);
  }
  else
  {
    fread(&MOD_BUS_Reg, 1, sizeof(MOD_BUS_REG), modbus_bin);
    fclose(modbus_bin);
    if(MOD_BUS_Reg.MOD_REG_MAGIC_WORD != DEFAULT_MOD_BUS_Reg.MOD_REG_MAGIC_WORD)
    {
      set_to_default = 1;
      printf("MOD_REG_MAGIC_WORD error ! %x\r\n", MOD_BUS_Reg.MOD_REG_MAGIC_WORD);
    }
    else
    {
      printf("LoadModbusReg ok ! \r\n");
    }
  }
  if(set_to_default)
  {
    modbus_bin = fopen("modbus_reg.bin", "wb+");
    memcpy(&MOD_BUS_Reg, &DEFAULT_MOD_BUS_Reg, sizeof(MOD_BUS_REG));
    fwrite(&MOD_BUS_Reg, 1, sizeof(MOD_BUS_REG), modbus_bin);
    fclose(modbus_bin);
  }

  SaveModbusReg_ms = GetCurrentTimeMs();
  return 0;
}

void SaveModbusReg_Task(void)
{
  s64 cur_ms = GetCurrentTimeMs(); 
  static int save_times = 0;
  if((SaveModbusReg_ms + SAVE_ONCE_BETWEEN_MS) < cur_ms)
  {
    printf("---- 1 minute ----\r\n");
    SaveModbusReg_ms = cur_ms;
    if(MOD_BUS_REG_FreshFlag) 
    {
      FILE* modbus_bin;
      MOD_BUS_REG_FreshFlag = 0;
      modbus_bin = fopen("modbus_reg.bin", "wb+");
      if (modbus_bin)
      {
        fwrite(&MOD_BUS_Reg, 1, sizeof(MOD_BUS_REG), modbus_bin);
        fclose(modbus_bin);
        printf("SaveModbusReg once, total = %d\r\n", ++save_times);
      }
      else
      {
        printf("SaveModbusReg Error ! \r\n");
      }
    }
  }
}

void Analysis_Receive_From_Master(u8 data, MODBUS* A8_Modbus, MOD_BUS_REG* MOD_BUS_Reg, int fd_radio)
{
    u8 receive_CRC_H = 0;
    u8 receive_CRC_L = 0;
    switch(A8_Modbus->machine_state)//初始化 默认 为 00;
    {
        case 0x00: 
        {
            if(data == MOD_BUS_Reg->SLAVE_ADDR)//从机地址
            {
                A8_Modbus->machine_state = 0x01;//从机地址可变，通过A8更改。
                A8_Modbus->data_index = 0;
                A8_Modbus->Probe_Slave_Addr = data;
                A8_Modbus->data[A8_Modbus->data_index++] = data;
            }
            else
            {
                A8_Modbus->machine_state = 0x0B;//缓冲数据区域清零要处理，中间数据为01，误认为是要从机地址。
                A8_Modbus->data_index = 0;
            }  
        } break;
        case 0x01:
        {	 
            A8_Modbus->data[A8_Modbus->data_index++] = data;
            if(data == CMD_ModBus_Read) //执行读取单个或多个寄存器命令  0x04 
            {
                A8_Modbus->machine_state = 0x02; 
                A8_Modbus->ModBus_CMD = CMD_ModBus_Read;
                A8_Modbus->read_receive_timer = 0;
            }
            else if(data == CMD_ModBus_Write)//执行写单个寄存器命令   0x06
            {
                A8_Modbus->machine_state = 0x03;
                A8_Modbus->ModBus_CMD = CMD_ModBus_Write;
                A8_Modbus->write_one_receive_timer = 0;
            }
            else if(data == CMD_ModBus_WriteMore)//执行写多个寄存器命令    0x16
            {
                A8_Modbus->machine_state = 0x04;
                A8_Modbus->ModBus_CMD = CMD_ModBus_WriteMore;
                A8_Modbus->write_more_receive_timer = 0;
            }    
            else
            { 
                A8_Modbus->machine_state = 0x0A;
                A8_Modbus->ModBus_CMD = data;
                A8_Modbus->err_state = 0x01;  // 功能码错误
            }
        } break;
        case 0x02: 
        {    
            //接收到读功能的地址和字节数
            A8_Modbus->data[A8_Modbus->data_index++] = data;
            A8_Modbus->read_receive_timer++;
            if( A8_Modbus->read_receive_timer == 4 )
            {
                A8_Modbus->machine_state = 0x06;  
                A8_Modbus->Read_Register_Addr = A8_Modbus->data[A8_Modbus->data_index-4]*256 + A8_Modbus->data[A8_Modbus->data_index-3];
                A8_Modbus->Read_Register_Num = A8_Modbus->data[A8_Modbus->data_index-2]*256 + A8_Modbus->data[A8_Modbus->data_index-1];
                A8_Modbus->read_receive_timer = 0;
            } 
        }break;
        case 0x03: 
        {   //接收到写功能地址和数据(单个字节)
            A8_Modbus->data[A8_Modbus->data_index++] = data;
            A8_Modbus->write_one_receive_timer++;
            if(A8_Modbus->write_one_receive_timer == 4)
            {
                A8_Modbus->machine_state = 0x07;
                A8_Modbus->Write_Register_Addr = A8_Modbus->data[A8_Modbus->data_index-4]*256 + A8_Modbus->data[A8_Modbus->data_index-3];
                A8_Modbus->Write_Register_Data_One = A8_Modbus->data[A8_Modbus->data_index-2]*256 + A8_Modbus->data[A8_Modbus->data_index-1];
                A8_Modbus->write_one_receive_timer = 0;
            }	 	 
        }break;
        case 0x04: 
        {   //接收到写功能地址和数据个数(多个字节)
            A8_Modbus->data[A8_Modbus->data_index++] = data;
            A8_Modbus->write_more_receive_timer++;
            if(A8_Modbus->write_more_receive_timer == 4)
            {
                A8_Modbus->machine_state = 0x08;
                A8_Modbus->write_more_receive_timer = 0;
                A8_Modbus->Write_Register_Addr = A8_Modbus->data[A8_Modbus->data_index-4]*256 + A8_Modbus->data[A8_Modbus->data_index-3];
                A8_Modbus->Write_Register_Num = (A8_Modbus->data[A8_Modbus->data_index-2]*256 + A8_Modbus->data[A8_Modbus->data_index-1]) * 2;
            }			      	 
        }break;
        case 0x06: 
        {   //读操作单一或则多个寄存器判断
            A8_Modbus->data[A8_Modbus->data_index++] = data;	
            A8_Modbus->read_receive_timer++;
            if(A8_Modbus->read_receive_timer == 2)
            {			
              u16 cal_crc;
                //计算接收到的数据 CRC结果。
                cal_crc=ModBus_CRC16_Calculate(A8_Modbus->data, 6);
                //判断接收到的CRC数据与计算的是否相同。
                receive_CRC_L = A8_Modbus->data[A8_Modbus->data_index-2];
                receive_CRC_H = A8_Modbus->data[A8_Modbus->data_index-1];
                if(((cal_crc>>8) == receive_CRC_H) 
                   && ((cal_crc&0xff) == receive_CRC_L) )
                {
                    A8_Modbus->err_state = 0x00;//CRC校验正确 
                    //printf("uart2_rx\n");
                    AckModBusReadReg(A8_Modbus->Read_Register_Addr,A8_Modbus->Read_Register_Num, fd_radio);
                }
                else	  
                {
                    A8_Modbus->err_state = 0x04;
                    AckModBusCrcError(CMD_ModBus_Read, fd_radio);
                }   
                A8_Modbus->data_index = 0;  
                A8_Modbus->read_receive_timer = 0;  
                A8_Modbus->machine_state = 0x00;
            }
        }break;
        case 0x07: 
        {   //写操作  单一寄存器
            A8_Modbus->data[A8_Modbus->data_index++] = data;	
        	  A8_Modbus->write_one_receive_timer++;
            if(A8_Modbus->write_one_receive_timer == 2)
            {
              u16 cal_crc;
                //计算接收到的数据 CRC结果。
                cal_crc=ModBus_CRC16_Calculate(A8_Modbus->data,6);
            	//判断接收到的CRC数据与计算的是否相同。
                receive_CRC_L = A8_Modbus->data[A8_Modbus->data_index-2];
                receive_CRC_H = A8_Modbus->data[A8_Modbus->data_index-1];
                if(((cal_crc>>8) == receive_CRC_H) 
                   && ((cal_crc&0xff) == receive_CRC_L) )
                {
                    A8_Modbus->err_state = 0x00;//CRC校验正确 		
                    AckModBusWriteOneReg(A8_Modbus->Write_Register_Addr,A8_Modbus->Write_Register_Data_One, fd_radio);
                }
                else	  
                {
                    A8_Modbus->err_state = 0x04;//CRC校验错误    
                    AckModBusCrcError(CMD_ModBus_Write, fd_radio);
                }

                A8_Modbus->machine_state = 0x00;
                A8_Modbus->data_index = 0;  
                A8_Modbus->write_one_receive_timer = 0;  					
            }
        }break;
        case 0x08: 
        {   //写入连续多个寄存器   
            A8_Modbus->data[A8_Modbus->data_index++] = data;
            A8_Modbus->Write_Register_Num--;					   
            if(A8_Modbus->Write_Register_Num == 0)
            {
                A8_Modbus->machine_state = 0x09;
                A8_Modbus->write_more_receive_timer = 2;					   
            }   
        }break;
        case 0x09: 
        {    
            A8_Modbus->data[A8_Modbus->data_index++] = data;
            A8_Modbus->write_more_receive_timer--;					   
            if(A8_Modbus->write_more_receive_timer == 0)
            {
              u16 cal_crc;
                //计算接收到的数据 CRC结果。
                cal_crc=ModBus_CRC16_Calculate(A8_Modbus->data, ((A8_Modbus->data[4]*256 + A8_Modbus->data[5])*2 + 6));
                //判断接收到的CRC数据与计算的是否相同。
                receive_CRC_L = A8_Modbus->data[A8_Modbus->data_index-2];
                receive_CRC_H = A8_Modbus->data[A8_Modbus->data_index-1];	                   
                if(((cal_crc>>8) == receive_CRC_H) && 
                   ((cal_crc&0xff) == receive_CRC_L) )
                {
                    A8_Modbus->err_state = 0x00;//CRC校验正确 	
                    A8_Modbus->Write_Register_Num = (A8_Modbus->data[4]*256 + A8_Modbus->data[5]);
                    AckModBusWriteMultiReg(A8_Modbus->Write_Register_Addr,A8_Modbus->Write_Register_Num,&A8_Modbus->data[6], fd_radio);
                }
                else	  
                {
                    A8_Modbus->err_state = 0x04;  
                    AckModBusCrcError(CMD_ModBus_WriteMore, fd_radio);
                }
                A8_Modbus->machine_state = 0x00;
                A8_Modbus->data_index = 0;  
                A8_Modbus->write_more_receive_timer = 0;				   
            }   
        }break;
        case 0x0A:
        {
            AckModBusFunctionError(A8_Modbus->ModBus_CMD, fd_radio);
            A8_Modbus->machine_state = 0x00;
        }break;
        case 0x0B:
        {//处理第一个字节不是地址的情况，等待数据流结束，重新同步
          //这里什么都不做，等待超时，后状态机自动恢复
        }break;
    }
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

u8 AckModBusCrcError(u8 CMD_ModBus, int fd_radio)
{
  u8 Send_Data_A8_array[256];
  u16 index=0;
  u16 cal_crc;
  Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
  Send_Data_A8_array[index++]=CMD_ModBus;
  Send_Data_A8_array[index++]=crc_err;
  cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
  Send_Data_A8_array[index++]=cal_crc&0xFF;
  Send_Data_A8_array[index++]=cal_crc>>8;

  Radio_Send(fd_radio, (char*)Send_Data_A8_array, index, RADIO_ACK_DELAY_MS);
  return 0;  
}

u8 AckModBusFunctionError(u8 CMD_ModBus, int fd_radio)
{
  u8 Send_Data_A8_array[256];
  u16 index=0;
  u16 cal_crc;
  Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
  Send_Data_A8_array[index++]=CMD_ModBus;
  Send_Data_A8_array[index++]=illegal_function;
  cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
  Send_Data_A8_array[index++]=cal_crc&0xff;
  Send_Data_A8_array[index++]=cal_crc>>8;

  Radio_Send(fd_radio, (char*)Send_Data_A8_array, index, RADIO_ACK_DELAY_MS);
  return 0;  
}

/*读取寄存器*/
u8 AckModBusReadReg(u16 reg_addr, u16 reg_num, int fd_radio)
{
  u8 Send_Data_A8_array[256];
  u16 index=0;
  u16 loop;
  if((reg_addr <= (MOD_BUS_REG_NUM - 1)) && ((reg_addr + reg_num) <= MOD_BUS_REG_NUM) && (reg_num <= 32))
  {
    u16* pBuf = (u16*)(&MOD_BUS_Reg) + reg_addr;
    u16 cal_crc;

    Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
    Send_Data_A8_array[index++]=CMD_ModBus_Read;
    Send_Data_A8_array[index++]=(reg_num<<1)>>8;//byte length ,MSB
    Send_Data_A8_array[index++]=(reg_num<<1)&0xFF;//byte length ,LSB
    for(loop=0;loop<reg_num;loop++)
    {
      Send_Data_A8_array[index++]=pBuf[loop]>>8;//MSB
      Send_Data_A8_array[index++]=pBuf[loop]&0xFF;//LSB
    }
    cal_crc = ModBus_CRC16_Calculate(Send_Data_A8_array , index);
    Send_Data_A8_array[index++]=cal_crc&0xFF;
    Send_Data_A8_array[index++]=cal_crc>>8;

    Radio_Send(fd_radio, (char*)Send_Data_A8_array, index, RADIO_ACK_DELAY_MS);
    return 1;
  }
  else
  {//数据错误、超出范围 illegal_data;Return-Code=0x03
    u16 cal_crc;
    Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
    Send_Data_A8_array[index++]=CMD_ModBus_Read;
    Send_Data_A8_array[index++]=illegal_register;
    cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
    Send_Data_A8_array[index++]=cal_crc&0xFF;
    Send_Data_A8_array[index++]=cal_crc>>8;

    Radio_Send(fd_radio, (char*)Send_Data_A8_array, index, RADIO_ACK_DELAY_MS);
  }
  return 0;
}


u8 AckModBusWriteOneReg(u16 reg_addr,u16 reg_value, int fd_radio)
{
  u8 Send_Data_A8_array[256];
  u16 index=0;
  u8 return_code=return_OK;
  u16 cal_crc;

#if (0)
  switch(reg_addr)
  { 
  case 0x02://设置波特率
    {
      if((reg_value>=1) && (reg_value<=8))
      {
        MOD_BUS_Reg.COMM_BD=reg_value;
        MOD_BUS_REG_FreshFlag=1;
        return_code=return_OK;
      }
      else
      {
        return_code=illegal_data;
      }
    }
    break;    
  case 0x03://设置从机地址
    {
      if(reg_value!=MOD_BUS_Reg.SLAVE_ADDR)
      {
        MOD_BUS_Reg.SLAVE_ADDR=reg_value;
        MOD_BUS_REG_FreshFlag=1;
      }
      return_code=return_OK;
    }
    break;   
  default:
    return_code=illegal_register;
  }
#else
  if(reg_addr == 0x20)
  {
    if(reg_value != MOD_BUS_Reg.VEHICLE_CONTROL)
    {
      if((reg_value != 0) && (MOD_BUS_Reg.MAP_NUM != 0) && (MOD_BUS_Reg.MAP_INDEX > 0) && (MOD_BUS_Reg.MAP_INDEX <= MOD_BUS_Reg.MAP_NUM))
      {
        MOD_BUS_Reg.VEHICLE_CONTROL = reg_value;
        return_code = return_OK;
      }
      else if(reg_value == 0)
      {
        MOD_BUS_Reg.VEHICLE_CONTROL = reg_value;
        return_code = return_OK;
      }
      else return_code = illegal_data;
    }
    else if(reg_value == 0x0)
    {
      MOD_BUS_Reg.VEHICLE_CONTROL = reg_value;
      return_code = return_OK;
    }
    else
    {
      return_code = illegal_data;
    }
  }
  else if(reg_addr == 0x21)
  {
    MOD_BUS_Reg.VEHICLE_TEST_CONTROL = reg_value;
    return_code = return_OK;
  }
  else if(reg_addr <= (MOD_BUS_REG_NUM - 1))
  {
    u16* pBuf = (u16*)(&MOD_BUS_Reg) + reg_addr;
    *pBuf = reg_value;
    MOD_BUS_REG_FreshFlag=1;
    return_code=return_OK;
  }
  else
  {
    return_code=illegal_register;
  }
#endif
  //回复用户
  Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
  Send_Data_A8_array[index++]=CMD_ModBus_Write;
  Send_Data_A8_array[index++]=return_code;
  cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
  Send_Data_A8_array[index++]=cal_crc&0xff;
  Send_Data_A8_array[index++]=cal_crc>>8;

  Radio_Send(fd_radio, (char*)Send_Data_A8_array, index, RADIO_ACK_DELAY_MS);
  return 0;    
}

u8 AckModBusWriteMultiReg(u16 reg_addr, u16 reg_num, u8* pData, int fd_radio)
{
  u8 Send_Data_A8_array[256];
  u16 index=0;
  u8 return_code=return_OK;  
  u16 cal_crc, loop;

  if(reg_addr == 0x28)
  {
    if(reg_num == 4)
    {
      float longti,lati;
      u8* ptr;
      u16 H16,L16;
      ptr = (u8*) &longti;
      ptr[0] = pData[3];
      ptr[1] = pData[2];
      ptr[2] = pData[1];
      ptr[3] = pData[0];
      ptr = (u8*) &lati;
      ptr[0] = pData[7];
      ptr[1] = pData[6];
      ptr[2] = pData[5];
      ptr[3] = pData[4];

      if((longti >= 0.0) && (longti <= 180.0) && (lati >= 0.0) && (lati <= 90.0)) 
      {        
        MOD_BUS_Reg.ORIGIN_LONGTI[0] = ((u16)pData[0] << 8) | (u16)pData[1];
        MOD_BUS_Reg.ORIGIN_LONGTI[1] = ((u16)pData[2] << 8) | (u16)pData[3];    
        MOD_BUS_Reg.ORIGIN_LATI[0] = ((u16)pData[4] << 8) | (u16)pData[5];
        MOD_BUS_Reg.ORIGIN_LATI[1] = ((u16)pData[6] << 8) | (u16)pData[7];
        init_COORDINATE_flag = 1;
        MOD_BUS_REG_FreshFlag=1;
        return_code=return_OK; 
      }
      else
      {
        return_code=illegal_data;
      }
    }
    else 
    {
      return_code=illegal_data;
    }
  }
  else if((reg_addr <= (MOD_BUS_REG_NUM - 1)) && ((reg_addr + reg_num) <= MOD_BUS_REG_NUM) && (reg_num <= 32))
  {
    u16* pBuf = (u16*)(&MOD_BUS_Reg) + reg_addr;
    for(loop = 0; loop < reg_num; loop++)
    {
      pBuf[loop] = ((u16)pData[loop*2] << 8) | ((u16)pData[loop*2 + 1] << 0);
    }
    
    MOD_BUS_REG_FreshFlag=1;
    return_code=return_OK;
  }
  else
  {
    return_code=illegal_register;
  }
  
  //回复用户
  Send_Data_A8_array[index++]=MOD_BUS_Reg.SLAVE_ADDR;
  Send_Data_A8_array[index++]=CMD_ModBus_WriteMore;
  Send_Data_A8_array[index++]=return_code;
  cal_crc=ModBus_CRC16_Calculate(Send_Data_A8_array , index);
  Send_Data_A8_array[index++]=cal_crc&0xff;
  Send_Data_A8_array[index++]=cal_crc>>8;
  
  Radio_Send(fd_radio, (char*)Send_Data_A8_array, index, RADIO_ACK_DELAY_MS);
  return 0;      
}



int Radio_Send(int fd, char *send_buf, int data_len, int delay_ms) 
{
  RADIO_INFOR_CONTENT* pRADIO_INFOR_CONTENT;
  pRADIO_INFOR_CONTENT = RADIO_INFOR_CONTENT_List + RADIO_INFOR_CONTENT_LIST_InIndex;
  pRADIO_INFOR_CONTENT->fd = fd;
  pRADIO_INFOR_CONTENT->delay_ms = delay_ms; 
  pRADIO_INFOR_CONTENT->data_len = (u8) data_len; 
  memcpy(pRADIO_INFOR_CONTENT->data, send_buf, pRADIO_INFOR_CONTENT->data_len);
  pRADIO_INFOR_CONTENT->timestamp = GetCurrentTimeMs();
  RADIO_INFOR_CONTENT_LIST_InIndex += 1;  
  RADIO_INFOR_CONTENT_LIST_InIndex &= (RADIO_INFOR_CONTENT_LIST_NUM_MASK);
  
  /*
  printf("Radio_Send, %d, %lld, %lld\r\n", 
    pRADIO_INFOR_CONTENT->data_len, 
    pRADIO_INFOR_CONTENT->timestamp, 
    pRADIO_INFOR_CONTENT->delay_ms);
  */
}

void Radio_Send_Task(void)
{
  if(RADIO_INFOR_CONTENT_LIST_InIndex != RADIO_INFOR_CONTENT_LIST_OutIndex)
  {
    RADIO_INFOR_CONTENT* pRADIO_INFOR_CONTENT = RADIO_INFOR_CONTENT_List + RADIO_INFOR_CONTENT_LIST_OutIndex;
    if((pRADIO_INFOR_CONTENT->timestamp + pRADIO_INFOR_CONTENT->delay_ms) <= GetCurrentTimeMs())
    {
      UART0_Send(pRADIO_INFOR_CONTENT->fd, (char*)pRADIO_INFOR_CONTENT->data, pRADIO_INFOR_CONTENT->data_len);
      RADIO_INFOR_CONTENT_LIST_OutIndex += 1;  
      RADIO_INFOR_CONTENT_LIST_OutIndex &= (RADIO_INFOR_CONTENT_LIST_NUM_MASK);
    }
  }
}


