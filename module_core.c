#include "module_core.h"
#include <stdio.h>      
#include <stdlib.h>    
#include <math.h>
#include <string.h>

MODULE_CORE_PARAM* MODULE_CORE_Param = NULL;

MODULE_CORE_PARAM* MODULE_CORE_Init(void)
{
  MODULE_CORE_PARAM* param = (MODULE_CORE_PARAM*) malloc(sizeof(MODULE_CORE_PARAM));
  if(param != NULL)
  {
    memset(param, 0, sizeof(MODULE_CORE_PARAM));
  }
  return param;
}

void MODULE_CORE_Release(MODULE_CORE_PARAM* param)
{
  if(param != NULL)
  {
    Coordinate_Release(param->coo);

    free(param);
  }
}
