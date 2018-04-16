#include "time.h"
#include "stdio.h"
#include "stdlib.h"


//---- LOG 记录初始化 ----//
FILE* Log_Init(void)
{
  char file_name[256];
  struct timespec time;
  struct tm nowtime;
  clock_gettime(CLOCK_REALTIME, &time);
  localtime_r(&time.tv_sec, &nowtime);

  sprintf(file_name, "log_%04d_%02d%02d_%02d%02d%02d.txt", nowtime.tm_year + 1900, nowtime.tm_mon + 1, nowtime.tm_mday, nowtime.tm_hour, nowtime.tm_min, nowtime.tm_sec);
  
  FILE* fd = fopen(file_name,"wt+");
  if(fd == NULL)
  {
    printf("Open Log File Error !\n");
  }
  else
  {
    //fprintf(fd, "--------------------------------\n");
  }

  return fd;
}

void Log_Release(FILE* fd)
{
  if(fd != NULL)
  {
    fclose(fd);
  }
}

