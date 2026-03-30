#include "data.h"
#include "hal.h"

Data::Data() : rx_pos(0), rx_buf(), tx_checksum()
{
}

bool Data::recv_heartbeat_frame(Heartbeat &out)
{
    int n = hal::uart.recv(&rx_buf[rx_pos], sizeof(rx_buf) - rx_pos);
    if (n < 0) {
        rx_pos = 0;
        return false;
    }

    rx_pos += n;

    while (1) {
        if (rx_pos > 0 && rx_buf[0] != 0x55) {
            shift_rx_buf(1);
            continue;
        }
        if (rx_pos > 1 && rx_buf[1] != 0xAA) {
            shift_rx_buf(2);
            continue;
        }
        if (rx_pos > 2 && rx_buf[2] != FRAME_TYPE_HEARTBEAT) {
            shift_rx_buf(3);
            continue;
        }
        switch (parse_heartbeat_frame(out)) {
            case PARSE_RESULT_WAIT:
                return false;
            case PARSE_RESULT_SUCCESS:
                return true;
            case PARSE_RESULT_FAILURE:
                continue;
        }
    }
}

Data::ParseResult Data::parse_heartbeat_frame(Heartbeat &out)
{
    const size_t frame_size = 11; // in bytes, including header and checksum
    static_assert(sizeof(rx_buf) >= frame_size, "a heartbeat frame cannot fit into the rx buffer, increase rx_buf size");
    if (rx_pos < frame_size) {
        return PARSE_RESULT_WAIT;
    }
    if (!verify_rx_checksum(frame_size)) {
        shift_rx_buf(frame_size);
        return PARSE_RESULT_FAILURE;
    }
    out.robot_mode = (RobotMode) rx_buf[3];
    out.team_color = (TeamColor) rx_buf[4];
    out.goal_zone = rx_buf[5];
    out.game_time = rx_buf[6] | ((uint32_t) rx_buf[7] << 8) | ((uint32_t) rx_buf[8] << 16) | ((uint32_t) rx_buf[9] << 24);
    shift_rx_buf(frame_size);
    return PARSE_RESULT_SUCCESS;
}

void Data::shift_rx_buf(size_t n)
{
    if (n < rx_pos) {
        for (size_t i = 0; n + i < rx_pos; ++i) {
            rx_buf[i] = rx_buf[n + i];
        }
        rx_pos -= n;
    } else {
        rx_pos = 0;
    }
}

bool Data::verify_rx_checksum(size_t n) const
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < n; ++i) {
        checksum += rx_buf[i];
    }
    return checksum == 0;
}

void Data::send_move_frame(int16_t delta_x, int16_t delta_y, int16_t delta_theta)
{
    send_frame_header(FRAME_TYPE_MOVE);
    send_u32(hal::time.get());
    send_i16(delta_x);
    send_i16(delta_y);
    send_i16(delta_theta);
    send_frame_checksum();
}

void Data::send_scan_frame(
    const Point2d *border_points, size_t border_point_count,
    const Point2d *obstacle_points, size_t obstacle_point_count
)
{
    send_frame_header(FRAME_TYPE_SCAN);
    send_u32(hal::time.get());
    send_u16(border_point_count);
    send_buf((const uint8_t *) border_points, 4 * border_point_count);
    send_u16(obstacle_point_count);
    send_buf((const uint8_t *) obstacle_points, 4 * obstacle_point_count);
    send_frame_checksum();
}

void Data::send_estimated_pose_frame(uint16_t x, uint16_t y, int16_t theta)
{
    send_frame_header(FRAME_TYPE_ESTIMATED_POSE);
    send_u32(hal::time.get());
    send_u16(x);
    send_u16(y);
    send_i16(theta);
    send_frame_checksum();
}

void Data::send_current_pose_frame(uint16_t x, uint16_t y, int16_t theta)
{
    send_frame_header(FRAME_TYPE_CURRENT_POSE);
    send_u32(hal::time.get());
    send_u16(x);
    send_u16(y);
    send_i16(theta);
    send_frame_checksum();
}

void Data::send_path_frame(const PathPoint *points, size_t point_count)
{
    send_frame_header(FRAME_TYPE_PATH);
    send_u32(hal::time.get());
    send_u16(point_count);
    send_buf((const uint8_t *) points, 2 * point_count);
    send_frame_checksum();
}

void Data::send_motor_frame(int16_t speed_a, int16_t speed_b, int16_t speed_c)
{
    send_frame_header(FRAME_TYPE_MOTOR);
    send_u32(hal::time.get());
    send_i16(speed_a);
    send_i16(speed_b);
    send_i16(speed_c);
    send_frame_checksum();
}

void Data::send_frame_header(FrameType frame_type)
{
    const uint8_t buf[3] = {
        0x55,
        0xAA,
        (uint8_t) (frame_type),
    };
    tx_checksum = 0;
    send_buf(buf, sizeof(buf));
}

void Data::send_frame_checksum()
{
    uint8_t x = -tx_checksum;
    hal::uart.send(&x, 1, true);
}

void Data::send_buf(const uint8_t *buf, size_t n)
{
    hal::uart.send(buf, n, false);
    for (size_t i = 0; i < n; ++i) {
        tx_checksum += buf[i];
    }
}

void Data::send_i16(int16_t x)
{
    send_buf((const uint8_t *) &x, 2);
}

void Data::send_u16(uint16_t x)
{
    send_buf((const uint8_t *) &x, 2);
}

void Data::send_u32(uint32_t x)
{
    send_buf((const uint8_t *) &x, 4);
}
