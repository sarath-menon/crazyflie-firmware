#define DEBUG_MODULE "HELLOWORLD"

extern "C" {
#include "cfassert.h"
#include "console.h"
#include "usb.h"
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

constexpr static uint8_t SYSLINK_MTU = 64;
static uint8_t sendBuffer[64];

static int usblinkSendPacket(CRTPPacket *p) {
  int dataSize;

  ASSERT(p->size < SYSLINK_MTU);

  sendBuffer[0] = p->header;

  if (p->size <= CRTP_MAX_DATA_SIZE) {
    memcpy(&sendBuffer[1], p->data, p->size);
  }
  dataSize = p->size + 1;

  // ledseqRun(&seq_linkDown);

  return usbSendData(dataSize, sendBuffer);
}

void appMain() {

  static CRTPPacket msg;

  while (1) {
    vTaskDelay(M2T(1000));
    // DEBUG_PRINT("Hello World!\n");

    uint8_t size = 1;
    msg.size = size;

    // uint8_t data = 'h';
    const char str[20] = {"selva\n"};

    // msg.data[0] = data;
    // // usblinkSendPacket(&msg);
    // crtpSendPacket(&msg);

    // consolePutchar(data);
    consolePuts(str);
  }
}
