#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "data_bw16_hal.h"

static FILE *serial_in;
static FILE *serial_out;

void data_hal_init()
{
    serial_in = fopen("serial/bw16_rx", "r");
    if (serial_in == NULL) {
        perror("cannot open serial input file");
        exit(1);
    }
    serial_out = fopen("serial/bw16_tx", "w");
    if (serial_out == NULL) {
        perror("cannot create serial output file");
        exit(1);
    }
}

void data_hal_send(const uint8_t *buf, size_t n, bool flush)
{
    for (size_t i = 0; i < n; ++i) {
        fprintf(serial_out, "%02X", buf[i]);
        if (i == n - 1 && flush) {
            fputc('\n', serial_out);
        } else {
            fputc(' ', serial_out);
        }
    }
}

int data_hal_recv(uint8_t *buf, size_t n)
{
    for (size_t i = 0; i < n; ++i) {
        int x;
        if (feof(serial_in)) {
            exit(0);
        }
        if (fscanf(serial_in, "%x", &x) != 1) {
            return -1;
        }
        buf[i]  =x;
    }
    return n;
}

uint32_t data_hal_time()
{
    static uint32_t t = 0;
    ++t;
    return t;
}