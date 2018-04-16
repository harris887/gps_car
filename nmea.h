#ifndef NMEA_TO_TAIP_
#define NMEA_TO_TAIP_

#define SEMICOLON   ','    
#define ASTERISK    '*'   

typedef	struct 
{ 
    int satid;      //������� 
    int elevation;  //�������ǣ�00 - 90����
    int azimuth;    //���Ƿ�λ�ǣ�00 - 359����
    int snr;        //����ȣ�00��99��dbHz 
} SatelliteInfo; 


typedef  struct
{
    char isvalid;  //GPS ��λ��־   0=δ��λ��1=�Ѷ�λ
    int hh,mm,ss,ms;

    int DD, MM, YY;
    double latitude;
    double latitude_InM;
    double latitude_InM_RTK_Fixed;
    unsigned char latNS;    
    double longitude;
    double longitude_InM;
    double longitude_InM_RTK_Fixed;
    unsigned char   lgtEW;
    float speed;       //�����ٶȣ�GPS�����λ��(Knots), �Ѿ�ת��λKM/H
    float direction;   //��λ�ǣ��� �����汱Ϊ�ο�

    double  altitude;     //���θ߶�
    unsigned char altitudeunit;       //���ε�λ

    unsigned char  FixMode;     //GPS״̬��0=δ��λ��1=�ǲ�ֶ�λ��2=��ֶ�λ��3=��ЧPPS��6=���ڹ���
    unsigned char GSA_mode1;//��λģʽ��A=�Զ��ֶ�2D/3D��M=�ֶ�2D/3D 
    unsigned char GSA_mode2;//��λ���ͣ�1=δ��λ��2=2D��λ��3=3D��λ 

    float PDOP;          //�ۺ�λ�þ�������
    float HDOP;          //ˮƽ��������
    float VDOP;          //��ֱ�������� 
    unsigned long  ageOfDiff;//���ʱ�䣨�����һ�ν��յ�����źſ�ʼ��������������ǲ�ֶ�λ��Ϊ�գ� 
    unsigned int  diffStationID;//���վID��0000 - 1023��ǰ��λ��������0��������ǲ�ֶ�λ��Ϊ�գ�
     
    unsigned char usedsat[12];//��������������������
    unsigned char usedsatnum;  //����ʹ�õ�����������00 - 12��
    unsigned char allsatnum;  //��ǰ�ɼ�����������00 - 12��
    SatelliteInfo satinfo[38];
    float Yaw;
    float Yaw_RTK_Fixed;
    unsigned char  AVR_FixMode;

    //---------
    int GGA_Num;
    int PTNL_AVR_Num;
}GPSINFO;
extern GPSINFO gps;

extern int Current_HourSec;
extern unsigned char NMEA_parse(char *buf);   

#endif
