#pragma once

#include "../data_types.h"

template <typename T>
class DataBufferLock;

template <typename T, size_t N>
class DataBuffer {
public:
    const size_t LEN = N;
    DataBuffer();
    DataBufferLock<T> lock();
private:
    // semaphore id
    T buf[N];
};

template <typename T>
class DataBufferLock {
public:
    ~DataBufferLock();
    T *ptr() const;
    void release();
private:
    // semaphore id
    T *_ptr;
    DataBufferLock(T *ptr);

    template <typename, size_t>
    friend class DataBuffer;
};

struct DataFrame {
    struct Move {
        uint32_t t;
        int16_t delta_x; // mm
        int16_t delta_y; // mm
        int16_t delta_theta; // deg
    };

    struct Scan {
        uint32_t t;
        uint16_t border_point_count;
        uint16_t obstacle_point_count;
        DataBufferLock<Point2d> border_points;
        DataBufferLock<Point2d> obstacle_points;
    };

    struct EstimatedPose {
        uint32_t t;
        uint16_t x; // mm
        uint16_t y; // mm
        int16_t theta; // deg
    };

    struct CurrentPose {
        uint32_t t;
        uint16_t x; // mm
        uint16_t y; // mm
        int16_t theta; // deg
    };

    struct Path {
        uint32_t t;
        uint16_t point_count;
        DataBufferLock<PathPoint> points;
    };

    struct Motor {
        uint32_t t;
        int16_t speed_a; // mm/s
        int16_t speed_b; // mm/s
        int16_t speed_c; // mm/s
    };

    FrameType type;
    union {
        Move move;
        Scan scan;
        EstimatedPose estimated_pose;
        CurrentPose current_pose;
        Path path;
        Motor motor;
    };

    DataFrame();
    ~DataFrame();
};

class Data {
public:
    Data();

    void send_heartbeat(const Heartbeat &heartbeat);

    // this function blocks until it receives a valid data frame
    void recv_frame(DataFrame &out);

private:
    uint8_t rx_checksum;
    uint8_t tx_checksum;

    void send_frame_header(FrameType frame_type);
    void send_frame_checksum();
    void send_buf(const uint8_t *buf, size_t n);
    void send_u8(uint8_t x);
    void send_u32(uint32_t x);

    int recv_frame_header();
    bool recv_frame_checksum();
    bool recv_buf(uint8_t *buf, size_t n);
    bool recv_buf_with_size(uint8_t *buf, size_t n, size_t buf_size);
    bool recv_u8(uint8_t &out);
    bool recv_i16(int16_t &out);
    bool recv_u16(uint16_t &out);
    bool recv_u32(uint32_t &out);

    bool recv_move_frame(DataFrame &out);
    bool recv_scan_frame(DataFrame &out);
    bool recv_estimated_pose_frame(DataFrame &out);
    bool recv_current_pose_frame(DataFrame &out);
    bool recv_path_frame(DataFrame &out);
    bool recv_motor_frame(DataFrame &out);
};
