#ifndef NMEA_TO_TAIP_
#define NMEA_TO_TAIP_

#define SEMICOLON   ','    
#define ASTERISK    '*'   

typedef	struct 
{ 
    int satid;      //卫星序号 
    int elevation;  //卫星仰角（00 - 90）度
    int azimuth;    //卫星方位角（00 - 359）度
    int snr;        //信噪比（00－99）dbHz 
} SatelliteInfo; 


typedef  struct
{
    char isvalid;  //GPS 定位标志   0=未定位，1=已定位
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
    float speed;       //地面速度，GPS输出单位节(Knots), 已经转化位KM/H
    float direction;   //方位角，度 ，以真北为参考

    double  altitude;     //海拔高度
    unsigned char altitudeunit;       //海拔单位

    unsigned char  FixMode;     //GPS状态，0=未定位，1=非差分定位，2=差分定位，3=无效PPS，6=正在估算
    unsigned char GSA_mode1;//定位模式，A=自动手动2D/3D，M=手动2D/3D 
    unsigned char GSA_mode2;//定位类型，1=未定位，2=2D定位，3=3D定位 

    float PDOP;          //综合位置精度因子
    float HDOP;          //水平精度因子
    float VDOP;          //垂直精度因子 
    unsigned long  ageOfDiff;//差分时间（从最近一次接收到差分信号开始的秒数，如果不是差分定位将为空） 
    unsigned int  diffStationID;//差分站ID号0000 - 1023（前导位数不足则补0，如果不是差分定位将为空）
     
    unsigned char usedsat[12];//正在用来解算的卫星序号
    unsigned char usedsatnum;  //正在使用的卫星数量（00 - 12）
    unsigned char allsatnum;  //当前可见卫星总数（00 - 12）
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
