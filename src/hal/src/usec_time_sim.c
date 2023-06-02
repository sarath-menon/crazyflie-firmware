/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

#include "usec_time.h"

void initUsecTimer(void)
{
  return ;
}

/* usecTimestamp just to be able to used crtp_commander_high_level module */
uint64_t usecTimestamp(void)
{
  return (unsigned long) ((xTaskGetTickCount())*(1000000.0/configTICK_RATE_HZ)) ;
}