#include <string.h>
#include <stdio.h>
#include <stdlib.h> 
#include <math.h>  
#include "nmea.h"
#include "misc.h"
#include "config.h"

#define my_atoi   atoi
#define my_atof   atof


#define Vehicle_ID_len  10
#define MS_START_LEN  4

const char Vehicle_ID[Vehicle_ID_len+1]=";ID=1234;*";/*Vehicle_ID*/
int Current_HourSec=0;
char TAIBEI_DATE[7]={'0','0','0','0','0','0',0};//DDMMYY
char Month_Dat[12]={31,28,31,30,31,30,31,31,30,31,30,31};//2may 29
GPSINFO  gps;    //GPS��Ϣ

//======================================================================
//�� �� ��: split() 
//��    ��: �ַ����ֽ�
//��ڲ���: *buf���ַ����׵�ַ  s���ָ��ʶ��  char **left  �ֽ��Ŀ�ʼ��ַ
//���ڲ���: ��
//�� �� ֵ: �ֽ�����ַ�����ַ 
//====================================================================== 
char *split(char *buf,char s,char **left)   
{   

    char *p=buf,*ret=buf;
      
    if(buf == NULL || buf[0] == 0)   
    {   
        *left=NULL;   
        return NULL;   
    }   
   
 
    while(*p != 0 && *p != s && *p != '\r' && *p != '\n')   
    {   
        p++;   
    }   
   
    if(*p != 0)   
    {   
        *left=p+1;   
        *p=0;   
    }   
    else   
    {   
        *left=NULL;   
    }   
   
    return ret;   
}

const unsigned char Digits_sixteen[16]=
{ '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F' }; 

//======================================================================
//�� �� ��: NMEA_Check() 
//��    ��: �ַ���У��
//��ڲ���: char *sentence���ַ����׵�ַ char *cksum У�����ַ
//���ڲ���: ��
//�� �� ֵ: 0: ��ȷ 1 ������ȷ; 
//======================================================================    
int NMEA_Check(char *sentence,char *cksum)   
{   

    unsigned char *p=(unsigned char *)sentence,sum=0; 
    if(sentence == NULL || cksum == NULL)   
        return 1;    //error
  
    for(; *p != 0; p++)   
    {   
        sum ^= *p;   
    }   
   
    if(Digits_sixteen[sum >> 4] == cksum[0] && Digits_sixteen[sum & 0x0f] == cksum[1])   
        return 0;   
  
    return 1;   //error
}   
char HAS_latitude=0;//�Ƿ���
//======================================================================
//�� �� ��: NMEA_parse() 
//��    ��: GPS������
//��ڲ���: char *buf �ַ����׵�ַ
//���ڲ���: ��
//�� �� ֵ: 0: �����ȷ   1 ����䲻��ȷ; 
//====================================================================== 
unsigned char NMEA_parse(char *buf)   
{   
   int d,m,mm;
   unsigned char i;
   char *word,*left=buf+1;    
   static unsigned char msgcount=0,msgid=0,satcount=0;   //����GSV�õ��ı���
           //��ͨ�����õ����Ǳ��    
   unsigned char usedsatcount=0;
   char temp[20];

    if(buf[0] != '$')           return 1;  //error flag

    word=split(left,ASTERISK,&left); //'*'
    if(NMEA_Check(word,left) != 0)   return 1;  //checksun error

    left=word;  
       
    word=split(left,SEMICOLON,&left);  //',' 
    if((!strcmp(word,"GPGGA")) || (!strcmp(word,"GNGGA")))   
    {   
        //hh mm ss ms  
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        { 
          long sec;
          char gps_time[9]="00000000";
          sscanf(word,"%2d%2d%2d.%2d",&gps.hh,&gps.mm,&gps.ss,&gps.ms);//ascii conv to int   
          sec=(long)gps.hh*3600L+(long)gps.mm*60+(long)gps.ss;
        }   
   
        //latitude 0-90   
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {
          if(gps.FixMode != 0)
          {
            int d,m,mm;   
            sscanf(word,"%2d%2d.%8d",&d,&m,&mm);   
            gps.latitude=(double)d+(double)m/60.0d+(double)mm/6000000000.0d;
            gps.latitude_InM=(double)d*60.0d+(double)m+(double)mm*0.00000001d;
            //sprintf (temp,"%11.7f", gps.latitude); //xxx.yyyyyyy

          }
        }   
   
        //NORTH OR SOUTH  
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            if(word[0] == 'S')   
            {
               gps.latNS = 'S';
            }
            else//positive north)
            {
               gps.latNS = 'N';
            } 
         }    
   
        //longitude 0-180    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {
          if(gps.FixMode!=0)
          {
            int d,m,mm;   
            sscanf(word,"%3d%2d.%8d",&d,&m,&mm);   
            gps.longitude=(double)d+(double)m/60.0d+(double)mm/6000000000.0d;  
            gps.longitude_InM=(double)d*60.0d+(double)m+(double)mm*0.00000001d; 
            //sprintf (temp,"%11.7f", gps.longitude); //xxx.yyyyyyy
          }
        }   
   
        //east west    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            if(word[0] == 'W')
            {   
               gps.lgtEW = 'W';

            } 
            else
            {
               gps.lgtEW = 'E';
            }  
        }   
   
        //��λ��Ч�Լ���ʽ    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            gps.FixMode=my_atoi(word);
            switch(gps.FixMode)
            {
            case 1://GPS SPS Mode, fix valid
              break;
            case 2://Differential GPS, SPS Mode, fix valid
              break;
            case 4:
              gps.latitude_InM_RTK_Fixed = gps.latitude_InM;
              gps.longitude_InM_RTK_Fixed = gps.longitude_InM;
              break;
            default : ;
            }
        }   
   
        //��׽��������    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            gps.usedsatnum=my_atoi(word);   

        }   
   
        //�������    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
           gps.HDOP=my_atof(word); 
            
            //sscanf(word,"%f",&gps.HDOP);  
        }   
   
        //���θ߶�    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
           gps.altitude=my_atof(word);   
           //sprintf (temp,"%08.1f", gps.altitude);//aaaaaa.b //units:METER
           //sprintf (temp,"%010.2f", gps.altitude/0.3048);//units:ft
        }   
   
        //�߶ȵ�λ    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            gps.altitudeunit=word[0];   
        }   
        
        gps.GGA_Num += 1;
        
#if (SIMULATE_ENABLE) 
#else
        if((gps.GGA_Num & 0x1) == 0) PID_Flag = 1;
#endif
    }   
    else if(!strcmp(word,"GPGSV"))   
    {   
        //��Ϣ����    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            msgcount=my_atoi(word);   
        }   
   
        //��Ϣ���    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            msgid=my_atoi(word);   
        }   
   
        if(msgid == 1)   
        {   
            satcount=0;   
            //memset(satinfo,0,sizeof(SatelliteInfo)*38);   
        }   
   
        //��������    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            gps.allsatnum=my_atoi(word);   
        }   
   
       // printf("%s\n",left);   
        for(i=0;i<4;i++)   
        {   
            //���Ǳ��    
            word=split(left,SEMICOLON,&left);   
            if(word != NULL)   
            {   
                gps.satinfo[satcount].satid=my_atoi(word);   
            }   
   
            //��������    
            word=split(left,SEMICOLON,&left);   
            if(word != NULL)   
            {   
                gps.satinfo[satcount].elevation=my_atoi(word);   
            }   
   
            //���Ƿ�λ��    
            word=split(left,SEMICOLON,&left);   
            if(word != NULL)   
            {   
                gps.satinfo[satcount].azimuth=my_atoi(word);   
            }   
   
            //�����ź������    
            word=split(left,SEMICOLON,&left);   
            if(word != NULL)   
            {   
                gps.satinfo[satcount].snr=my_atoi(word);   
            }   
   
            satcount++;   
   
            if(word == NULL)   
                break;   
        }   
   
        if(msgid == msgcount )   
          ;   
    }   
    else if(!strcmp(word,"GPGSA"))   
    {   
        //��λģʽ1    
        word=split(left,SEMICOLON,&left);   //A-AUTO;M-MANEL
        if(word != NULL)   
        {   
            gps.GSA_mode1=word[0];   
        }   
   
        //��λģʽ2    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            gps.GSA_mode2=my_atoi(word);   
        }   
          
        for(i=0;i<12;i++)   
        {   
            word=split(left,SEMICOLON,&left);   
            if(word != NULL)   
            {   
                gps.usedsat[i]=my_atoi(word);   
                usedsatcount++;
            }   
        }   
   
        //λ�þ���ֵ    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            gps.PDOP=(float)my_atof(word);   
        }   
   
        //ˮƽ����ֵ    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            gps.HDOP=(float)my_atof(word);   
        }   
   
        //�߶Ⱦ���ֵ    
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            gps.VDOP=(float)my_atof(word);   
        }       
    }
    // $PTNL,AVR,094922.70,+269.0761,Yaw,+2.7909,Tilt,,,3.767,3,5.6,04*3D
    else if(!strcmp(word,"PTNL"))   
    {   
        //MSG_ID  
        word=split(left,SEMICOLON,&left);   
        if(word != NULL)   
        {   
            if(strcmp(word,"AVR") == 0) 
            {
                //UTC of vector fix
                word=split(left,SEMICOLON,&left);   
                if(word != NULL)   
                {   
                     
                }   
           
                //Yaw angle in degrees
                word=split(left,SEMICOLON,&left);   
                if(word != NULL)   
                {   
                    gps.Yaw=(float)my_atof(word);   
                } 
                
                word=split(left,SEMICOLON,&left);  
                word=split(left,SEMICOLON,&left); 
                word=split(left,SEMICOLON,&left); 
                word=split(left,SEMICOLON,&left); 
                word=split(left,SEMICOLON,&left); 
                word=split(left,SEMICOLON,&left); 
                word=split(left,SEMICOLON,&left); 
                if(word != NULL)   
                {   
                    gps.AVR_FixMode=my_atoi(word);
                    switch(gps.AVR_FixMode)
                    {
                    case 1://GPS SPS Mode, fix valid
                      break;
                    case 2:
                      break;
                    case 3:
                      gps.Yaw_RTK_Fixed = gps.Yaw;
                      break;
                    default : ;
                    }
                } 

 
                gps.PTNL_AVR_Num += 1;
            }
            //else if(strcmp(word,"VGK") == 0) 
            //else if(strcmp(word,"VHD") == 0) 
        }   
     
    }
    //==============����ֵ���� =================
    if(gps.latitude > 90.0)
      gps.latitude = 0.0;

        if(gps.longitude > 180.0)
      gps.longitude = 0.0;
   //=============����ֵ����ԭ����============
    return 1;   
}

