#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

namespace hal {
    class UART {
    public:
        UART();
        ~UART();
        void send(const uint8_t *buf, size_t n, bool flush);
        int recv(uint8_t *buf, size_t n);
    private:
        FILE *in;
        FILE *out;
    };

    extern UART uart;

    class Time {
    public:
        Time();
        uint32_t get();
    private:
        uint32_t t;
    };

    extern Time time;

    class File {
    public:
        File();
        ~File();
        bool open(const char *path);
        void close();
        void write(const uint8_t *buf, size_t n);
    private:
        FILE *f;
    };
}
