#define DEBUG_MODULE "HELLOWORLD"

extern "C" {
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "app.h"
#include "crtp.h"
#include "dds_app.h"
#include "debug.h"
#include "task.h"
}

void appMain() {

  static CRTPPacket msg;

  while (1) {
    vTaskDelay(M2T(1000));
    DEBUG_PRINT("Hello World!\n");

    uint8_t size = 1;
    msg.size = size;

    uint8_t data = 'a';

    msg.data[0] = data;
    crtpSendPacket(&msg);
  }
}
