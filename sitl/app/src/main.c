/* Personal configs */
#include "FreeRTOSConfig.h"

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

/* Project includes */
#include "config.h"
// #include "platform.h"
#include "system.h"
#include "stdio.h"
// #include "use.hc_time.h"

int main() {

  // Launch the system task that will initialize and start everything
  systemLaunch();

  // Start the FreeRTOS scheduler
  vTaskStartScheduler();

  // Should never reach this point!
  while (1)
    ;

  return 0;
}

/*
 * FreeRTOS assert name
 */
void vAssertCalled(unsigned long ulLine, const char *const pcFileName) {
  printf("ASSERT: %s : %d\n", pcFileName, (int)ulLine);
  while (1)
    ;
}

unsigned long ulGetRunTimeCounterValue(void) { return 0; }

void vConfigureTimerForRunTimeStats(void) { return; }

/* For memory management */
void vApplicationMallocFailedHook(void) {
  while (1)
    ;
}

#if (configCHECK_FOR_STACK_OVERFLOW > 0)
void vApplicationStackOverflowHook(TaskHandle_t xTask, char * pcTaskName)
{
  printf("Stack overflow\n");
  while(1);
}
#endif