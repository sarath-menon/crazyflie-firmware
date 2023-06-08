#pragma once

#include <device.h>
#include <unistd.h>
#include <uxr/client/client.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  size_t fd;
  struct device *uart_dev;
} dds_transport_params_t;

static dds_transport_params_t default_params;

bool dds_transport_open(struct uxrCustomTransport *transport);
bool dds_transport_close(struct uxrCustomTransport *transport);
size_t dds_transport_write(struct uxrCustomTransport *transport,
                           const uint8_t *buf, size_t len, uint8_t *err);
size_t dds_transport_read(struct uxrCustomTransport *transport, uint8_t *buf,
                          size_t len, int timeout, uint8_t *err);

#ifdef __cplusplus
}
#endif
