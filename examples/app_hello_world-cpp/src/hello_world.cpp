#define DEBUG_MODULE "HELLOWORLD"

extern "C" {
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app.h"
#include "debug.h"

#include "FreeRTOS.h"
#include "task.h"
}

void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  while (1) {
    vTaskDelay(M2T(2000));
    DEBUG_PRINT("Hello World!\n");
  }
}
