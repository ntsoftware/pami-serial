#include "data.h"
#include "hal.h"

static size_t rx_pos;
static uint8_t rx_buf[16];
static uint8_t tx_checksum;

static void send_frame_header(enum data_frame_type frame_type);
static void send_frame_checksum();
static void send_buf(const uint8_t *buf, size_t n);
static void send_i16(int16_t x);
static void send_u16(uint16_t x);
static void send_u32(uint32_t x);

static void shift_rx_buf(size_t n);
static bool verify_rx_checksum(size_t n);

enum parse_result {
    PARSE_WAIT,
    PARSE_SUCCESS,
    PARSE_FAILURE,
};

static enum parse_result parse_heartbeat_frame(struct data_heartbeat *out);

void data_init()
{
    rx_pos = 0;
}

bool data_recv_heartbeat_frame(struct data_heartbeat *out)
{
    int n = hal_uart_recv(&rx_buf[rx_pos], sizeof(rx_buf) - rx_pos);
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
        if (rx_pos > 2 && rx_buf[2] != DATA_TYPE_HEARTBEAT) {
            shift_rx_buf(3);
            continue;
        }
        switch (parse_heartbeat_frame(out)) {
            case PARSE_WAIT:
                return false;
            case PARSE_SUCCESS:
                return true;
            case PARSE_FAILURE:
                continue;
        }
    }
}

static enum parse_result parse_heartbeat_frame(struct data_heartbeat *out)
{
    const size_t frame_size = 11; // in bytes, including header and checksum
    static_assert(sizeof(rx_buf) >= frame_size, "a heartbeat frame cannot fit into the rx buffer, increase rx_buf size");
    if (rx_pos < frame_size) {
        return PARSE_WAIT;
    }
    if (!verify_rx_checksum(frame_size)) {
        shift_rx_buf(frame_size);
        return PARSE_FAILURE;
    }
    out->robot_mode = (enum robot_mode) rx_buf[3];
    out->team_color = (enum team_color) rx_buf[4];
    out->goal_zone = rx_buf[5];
    out->game_time = rx_buf[6] | ((uint32_t) rx_buf[7] << 8) | ((uint32_t) rx_buf[8] << 16) | ((uint32_t) rx_buf[9] << 24);
    shift_rx_buf(frame_size);
    return PARSE_SUCCESS;
}

static void shift_rx_buf(size_t n)
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

static bool verify_rx_checksum(size_t n)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < n; ++i) {
        checksum += rx_buf[i];
    }
    return checksum == 0;
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
    hal_uart_send(&x, 1, true);
}

static void send_buf(const uint8_t *buf, size_t n)
{
    hal_uart_send(buf, n, false);
    for (size_t i = 0; i < n; ++i) {
        tx_checksum += buf[i];
    }
}

static void send_i16(int16_t x)
{
    send_buf((const uint8_t *) &x, 2);
}

static void send_u16(uint16_t x)
{
    send_buf((const uint8_t *) &x, 2);
}

static void send_u32(uint32_t x)
{
    send_buf((const uint8_t *) &x, 4);
}
