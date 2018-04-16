#include <string.h>
#include <stdio.h>
#include <stdlib.h> 
#include <math.h>  

#ifndef PI
#define PI 3.1415926535898
#endif 

double Get_TwoPointDistance(double lati_01, double longti_01, double lati_02, double longti_02)
{
    double earth_r = 6371 * 1000.0;
    double lati_cycle_r = earth_r * cos((lati_01+lati_02) * (PI / 180.0) * 0.5 / 60.0 );//
    double dx = (longti_01 - longti_02) * (PI / 180.0) * (lati_cycle_r / 60.0);
    double dy = (lati_01 - lati_02) * (PI / 180.0) * (earth_r / 60.0);
    printf("lati_r = %lf, dx = %lf, dy = %lf \r\n", lati_cycle_r, dx, dy);
    return sqrt(dx * dx + dy * dy);
}



double jw[2][2] = 
{
  {116.420973,40.048134},
  {116.44131,40.048852},
};
void main(void)
{
  double dis = Get_TwoPointDistance(jw[0][1] * 60.0d, jw[0][0] * 60.0d, jw[1][1] * 60.0d, jw[1][0] * 60.0d);

  printf("dis = %lf\r\n", dis);
}
