#pragma once

#include "../data_types.h"

class Data {
public:
    Data();

    // this function returns true if a valid heartbeat frame is received in the out buffer
    bool recv_heartbeat_frame(Heartbeat &out);

    void send_move_frame(int16_t delta_x, int16_t delta_y, int16_t delta_theta);

    void send_scan_frame(
        const Point2d *border_points, size_t border_point_count,
        const Point2d *obstacle_points, size_t obstacle_point_count
    );

    void send_estimated_pose_frame(uint16_t x, uint16_t y, int16_t theta);
    void send_current_pose_frame(uint16_t x, uint16_t y, int16_t theta);

    void send_path_frame(const PathPoint *points, size_t point_count);

    void send_motor_frame(int16_t speed_a, int16_t speed_b, int16_t speed_c);

private:
    size_t rx_pos;
    uint8_t rx_buf[16];
    uint8_t tx_checksum;

    void send_frame_header(FrameType frame_type);
    void send_frame_checksum();
    void send_buf(const uint8_t *buf, size_t n);
    void send_i16(int16_t x);
    void send_u16(uint16_t x);
    void send_u32(uint32_t x);

    void shift_rx_buf(size_t n);
    bool verify_rx_checksum(size_t n) const;

    enum ParseResult {
        PARSE_RESULT_WAIT,
        PARSE_RESULT_SUCCESS,
        PARSE_RESULT_FAILURE,
    };

    ParseResult parse_heartbeat_frame(Heartbeat &out);
};
