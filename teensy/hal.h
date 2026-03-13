#pragma once

#include <stddef.h>
#include <stdint.h>

void hal_init();
void hal_uart_send(const uint8_t *buf, size_t n, bool flush);
int hal_uart_recv(uint8_t *buf, size_t n);
uint32_t data_hal_time();