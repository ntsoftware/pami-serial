#pragma once

#include <stddef.h>
#include <stdint.h>

void data_hal_init();
void data_hal_send(const uint8_t *buf, size_t n, bool flush);
int data_hal_recv(uint8_t *buf, size_t n);
uint32_t data_hal_time();