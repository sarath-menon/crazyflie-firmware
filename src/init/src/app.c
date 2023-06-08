#include "app.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"

#include <uxr/client/client.h>
#include <uxr/client/transport.h>
#include <ucdr/microcdr.h>

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define RING_BUF_SIZE 2048

char uart_in_buffer[RING_BUF_SIZE];
char uart_out_buffer[RING_BUF_SIZE];

bool dds_transport_open(struct uxrCustomTransport * transport){}

bool dds_transport_close(struct uxrCustomTransport * transport){}

size_t dds_transport_write(struct uxrCustomTransport *transport,
                           const uint8_t *buf, size_t len, uint8_t *err){}

size_t dds_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){}