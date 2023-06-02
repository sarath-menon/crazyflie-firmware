/* Standard includes. */
extern "C" {

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Personal configs */
#include "FreeRTOSConfig.h"

/* Project includes */
#include "config.h"
// #include "platform.h"
}

void vTask1(void *pvParameters);
void vTask2(void *pvParameters);

int main(void) {
  xTaskCreate(&vTask1, "Task 1", 1024, NULL, 1, NULL);
  xTaskCreate(&vTask2, "Task 2", 1024, NULL, 1, NULL);

  vTaskStartScheduler();

  return 0;
}

void vTask1(void *pvParameters) {
  for (;;) {
    printf("Task 1\r\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void vTask2(void *pvParameters) {
  for (;;) {
    printf("Task 2\r\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
