#include "data.h"
#include "hal.h"

static uint8_t rx_checksum;
static uint8_t tx_checksum;

static void send_frame_header(enum data_frame_type frame_type);
static void send_frame_checksum();
static void send_buf(const uint8_t *buf, size_t n);
static void send_u8(uint8_t x);
static void send_u32(uint32_t x);

static int recv_frame_header();
static bool recv_frame_checksum();
static bool recv_buf(uint8_t *buf, size_t n);
static bool recv_u8(uint8_t *out);
static bool recv_i16(int16_t *out);
static bool recv_u16(uint16_t *out);
static bool recv_u32(uint32_t *out);

static bool recv_move_frame(struct data_frame *out);
static bool recv_scan_frame(struct data_frame *out);
static bool recv_estimated_pose_frame(struct data_frame *out);
static bool recv_current_pose_frame(struct data_frame *out);
static bool recv_path_frame(struct data_frame *out);
static bool recv_motor_frame(struct data_frame *out);

void data_send_heartbeat(const struct data_heartbeat &heartbeat)
{
    send_frame_header(DATA_TYPE_HEARTBEAT);
    send_u8(heartbeat.robot_mode);
    send_u8(heartbeat.team_color);
    send_u8(heartbeat.goal_zone);
    send_u32(heartbeat.game_time);
    send_frame_checksum();
}

void data_recv_frame(struct data_frame *out)
{
    while (1) {
        switch (recv_frame_header()) {
            case DATA_TYPE_MOVE:
                if (recv_move_frame(out)) return;
                break;
            case DATA_TYPE_SCAN:
                if (recv_scan_frame(out)) return;
                break;
            case DATA_TYPE_ESTIMATED_POSE:
                if (recv_estimated_pose_frame(out)) return;
                break;
            case DATA_TYPE_CURRENT_POSE:
                if (recv_current_pose_frame(out)) return;
                break;
            case DATA_TYPE_PATH:
                if (recv_path_frame(out)) return;
                break;
            case DATA_TYPE_MOTOR:
                if (recv_motor_frame(out)) return;
                break;
            default:
                continue;
        }
    }
}

static bool recv_move_frame(struct data_frame *out)
{
    uint32_t t;
    int16_t delta_x;
    int16_t delta_y;
    int16_t delta_theta;

    if (!recv_u32(&t)) return false;
    if (!recv_i16(&delta_x)) return false;
    if (!recv_i16(&delta_y)) return false;
    if (!recv_i16(&delta_theta)) return false;
    if (!recv_frame_checksum()) return false;

    out->type = DATA_TYPE_MOVE;
    out->move.t = t;
    out->move.delta_x = delta_x;
    out->move.delta_y = delta_y;
    out->move.delta_theta = delta_theta;
    return true;
}

static bool recv_scan_frame(struct data_frame *out)
{
    uint32_t t;
    uint16_t border_point_count;
    uint16_t obstacle_point_count;
    struct point2d *border_points = data_acquire_border_points();
    struct point2d *obstacle_points = data_acquire_obstacle_points();

    if (!recv_u32(&t)) goto err;

    if (!recv_u16(&border_point_count)) goto err;
    if (!recv_buf((uint8_t *) border_points, 4 * border_point_count)) goto err;

    if (!recv_u16(&obstacle_point_count)) goto err;
    if (!recv_buf((uint8_t *) obstacle_points, 4 * obstacle_point_count)) goto err;

    if (!recv_frame_checksum()) goto err;

    out->type = DATA_TYPE_SCAN;
    out->scan.t = t;
    out->scan.border_point_count = border_point_count;
    out->scan.border_points = border_points;
    out->scan.obstacle_point_count = obstacle_point_count;
    out->scan.obstacle_points = obstacle_points;
    return true;

err:
    data_release_border_points();
    data_release_obstacle_points();
    return false;
}

static bool recv_estimated_pose_frame(struct data_frame *out)
{
    uint32_t t;
    uint16_t x;
    uint16_t y;
    int16_t theta;

    if (!recv_u32(&t)) return false;
    if (!recv_u16(&x)) return false;
    if (!recv_u16(&y)) return false;
    if (!recv_i16(&theta)) return false;
    if (!recv_frame_checksum()) return false;

    out->type = DATA_TYPE_ESTIMATED_POSE;
    out->estimated_pose.t = t;
    out->estimated_pose.x = x;
    out->estimated_pose.y = y;
    out->estimated_pose.theta = theta;
    return true;
}

static bool recv_current_pose_frame(struct data_frame *out)
{
    uint32_t t;
    uint16_t x;
    uint16_t y;
    int16_t theta;

    if (!recv_u32(&t)) return false;
    if (!recv_u16(&x)) return false;
    if (!recv_u16(&y)) return false;
    if (!recv_i16(&theta)) return false;
    if (!recv_frame_checksum()) return false;

    out->type = DATA_TYPE_CURRENT_POSE;
    out->current_pose.t = t;
    out->current_pose.x = x;
    out->current_pose.y = y;
    out->current_pose.theta = theta;
    return true;
}

static bool recv_path_frame(struct data_frame *out)
{
    uint32_t t;
    uint16_t point_count;
    struct path_point *points = NULL;

    if (!recv_u32(&t)) goto err;

    if (!recv_u16(&point_count)) goto err;
    points = data_acquire_path_points();
    if (!recv_buf((uint8_t *) points, 2 * point_count)) goto err;

    if (!recv_frame_checksum()) goto err;

    out->type = DATA_TYPE_PATH;
    out->path.t = t;
    out->path.point_count = point_count;
    out->path.points = points;
    return true;

err:
    if (points != NULL) {
        data_release_path_points();
    }
    return false;
}

static bool recv_motor_frame(struct data_frame *out)
{
    uint32_t t;
    int16_t speed_a;
    int16_t speed_b;
    int16_t speed_c;

    if (!recv_u32(&t)) return false;
    if (!recv_i16(&speed_a)) return false;
    if (!recv_i16(&speed_b)) return false;
    if (!recv_i16(&speed_c)) return false;
    if (!recv_frame_checksum()) return false;

    out->type = DATA_TYPE_MOTOR;
    out->motor.t = t;
    out->motor.speed_a = speed_a;
    out->motor.speed_b = speed_b;
    out->motor.speed_c = speed_c;
    return true;
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
    if (hal_uart_recv(buf, n) == (int) n) {
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

static bool recv_i16(int16_t *out)
{
    return recv_buf((uint8_t *) out, 2);
}

static bool recv_u16(uint16_t *out)
{
    return recv_buf((uint8_t *) out, 2);
}

static bool recv_u32(uint32_t *out)
{
    return recv_buf((uint8_t *) out, 4);
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

static void send_u8(uint8_t x)
{
    send_buf(&x, 1);
}

static void send_u32(uint32_t x)
{
    send_buf((const uint8_t *) &x, 4);
}