#include "data.h"
#include "data_teensy.h"
#include "data_teensy_hal.h"

static uint8_t rx_checksum;
static uint8_t tx_checksum;

static void send_frame_header(enum data_frame_type frame_type);
static void send_frame_checksum();
static void send_buf(const uint8_t *buf, size_t n);
static void send_i16(int16_t x);
static void send_u16(uint16_t x);
static void send_u32(uint32_t x);

static int recv_frame_header();
static bool recv_frame_checksum();
static bool recv_buf(uint8_t *buf, size_t n);
static bool recv_u8(uint8_t *out);
static bool recv_u32(uint32_t *out);

void data_recv_heartbeat_frame(struct data_heartbeat *out)
{
    while (1) {
        if (recv_frame_header() == DATA_TYPE_HEARTBEAT) {
            uint8_t robot_mode;
            uint8_t team_color;
            uint8_t goal_zone;
            uint32_t game_time;

            if (!recv_u8(&robot_mode)) continue;
            if (!recv_u8(&team_color)) continue;
            if (!recv_u8(&goal_zone)) continue;
            if (!recv_u32(&game_time)) continue;

            if (recv_frame_checksum()) {
                out->robot_mode = (enum robot_mode) robot_mode;
                out->team_color = (enum team_color) team_color;
                out->goal_zone = goal_zone;
                out->game_time = game_time;
                return;
            }
        }
    }
}

void data_send_move_frame(int16_t delta_x, int16_t delta_y, int16_t delta_theta)
{
    send_frame_header(DATA_TYPE_MOVE);
    send_u32(data_hal_time());
    send_i16(delta_x);
    send_i16(delta_y);
    send_i16(delta_theta);
    send_frame_checksum();
}

void data_send_scan_frame(
    const struct point2d *border_points, size_t border_point_count,
    const struct point2d *obstacle_points, size_t obstacle_point_count
)
{
    send_frame_header(DATA_TYPE_SCAN);
    send_u32(data_hal_time());
    send_u16(border_point_count);
    send_buf((const uint8_t *) border_points, 4 * border_point_count);
    send_u16(obstacle_point_count);
    send_buf((const uint8_t *) obstacle_points, 4 * obstacle_point_count);
    send_frame_checksum();
}

void data_send_estimated_pose_frame(uint16_t x, uint16_t y, int16_t theta)
{
    send_frame_header(DATA_TYPE_ESTIMATED_POSE);
    send_u32(data_hal_time());
    send_u16(x);
    send_u16(y);
    send_i16(theta);
    send_frame_checksum();
}

void data_send_current_pose_frame(uint16_t x, uint16_t y, int16_t theta)
{
    send_frame_header(DATA_TYPE_CURRENT_POSE);
    send_u32(data_hal_time());
    send_u16(x);
    send_u16(y);
    send_i16(theta);
    send_frame_checksum();
}

void data_send_path_frame(const struct path_point *points, size_t point_count)
{
    send_frame_header(DATA_TYPE_PATH);
    send_u32(data_hal_time());
    send_u16(point_count);
    send_buf((const uint8_t *) points, 2 * point_count);
    send_frame_checksum();
}

void data_send_motor_frame(int16_t speed_a, int16_t speed_b, int16_t speed_c)
{
    send_frame_header(DATA_TYPE_MOTOR);
    send_u32(data_hal_time());
    send_i16(speed_a);
    send_i16(speed_b);
    send_i16(speed_c);
    send_frame_checksum();
}

static void send_frame_header(enum data_frame_type frame_type)
{
    const uint8_t buf[3] = {
        0x55,
        0xAA,
        (uint8_t) (frame_type),
    };
    tx_checksum = 0;
    send_buf(buf, sizeof(buf));
}

static void send_frame_checksum()
{
    uint8_t x = -tx_checksum;
    data_hal_send(&x, 1, true);
}

static void send_buf(const uint8_t *buf, size_t n)
{
    data_hal_send(buf, n, false);
    for (size_t i = 0; i < n; ++i) {
        tx_checksum += buf[i];
    }
}

static void send_i16(int16_t x)
{
    const uint8_t *buf = (const uint8_t *) &x;
    data_hal_send(buf, 2, false);
    tx_checksum += buf[0] + buf[1];
}

static void send_u16(uint16_t x)
{
    const uint8_t *buf = (const uint8_t *) &x;
    data_hal_send(buf, 2, false);
    tx_checksum += buf[0] + buf[1];
}

static void send_u32(uint32_t x)
{
    const uint8_t *buf = (const uint8_t *) &x;
    data_hal_send(buf, 4, false);
    tx_checksum += buf[0] + buf[1] + buf[2] + buf[3];
}

static int recv_frame_header()
{
    rx_checksum = 0;

    uint8_t x;
    if (!recv_u8(&x) || x != 0x55) return -1;
    if (!recv_u8(&x) || x != 0xAA) return -1;

    uint8_t y;
    if (recv_u8(&y)) {
        return y;
    } else {
        return -1;
    }
}

static bool recv_frame_checksum()
{
    uint8_t x;
    return recv_u8(&x) && rx_checksum == 0;
}

static bool recv_buf(uint8_t *buf, size_t n)
{
    if (data_hal_recv(buf, n) == (int) n) {
        for (size_t i = 0; i < n; ++i) {
            rx_checksum += buf[i];
        }
        return true;
    } else {
        return false;
    }
}

static bool recv_u8(uint8_t *out)
{
    return recv_buf(out, 1);
}

static bool recv_u32(uint32_t *out)
{
    return recv_buf((uint8_t *) out, 4);
}