#define _CRT_SECURE_NO_WARNINGS
#include "hal.h"

using namespace hal;

UART hal::uart;

UART::UART()
{
    in = fopen(HAL_RX_FILE, "r");
    if (in == NULL) {
        perror("cannot open serial input file");
        exit(1);
    }
    out = fopen(HAL_TX_FILE, "w");
    if (out == NULL) {
        perror("cannot create serial output file");
        exit(1);
    }
}

UART::~UART()
{
    fclose(in);
    fclose(out);
}

void UART::send(const uint8_t *buf, size_t n, bool flush)
{
    for (size_t i = 0; i < n; ++i) {
        fprintf(out, "%02X", buf[i]);
        if (i == n - 1 && flush) {
            fputc('\n', out);
        } else {
            fputc(' ', out);
        }
    }
}

int UART::recv(uint8_t *buf, size_t n)
{
    if (feof(in)) {
        exit(0);
    }
    for (size_t i = 0; i < n; ++i) {
        int x;
        if (fscanf(in, "%x", &x) != 1) {
            return i;
        }
        buf[i] = x;
    }
    return n;
}

Time hal::time;

Time::Time() : t(0)
{
}

uint32_t Time::get()
{
    ++t;
    return t;
}

File::File() : f(NULL)
{
}

File::~File()
{
    close();
}

bool File::open(const char *path)
{
    f = fopen(path, "wb");
    return f != NULL;
}

void File::close()
{
    if (f != NULL) {
        fclose(f);
        f = NULL;
    }
}

void File::write(const uint8_t *buf, size_t n)
{
    fwrite(buf, n, 1, f);
}
