#include "data.h"
#include "data_bw16.h"
#include "data_bw16_hal.h"

static uint8_t rx_checksum;
// static uint8_t tx_checksum;

static int recv_frame_header();
static bool recv_frame_checksum();
static bool recv_buf(uint8_t *buf, size_t n);
static bool recv_u8(uint8_t *out);
static bool recv_i16(int16_t *out);
static bool recv_u16(uint16_t *out);
static bool recv_u32(uint32_t *out);

static bool recv_move_frame(struct data_frame *frame);
static bool recv_scan_frame(struct data_frame *frame);
static bool recv_estimated_pose_frame(struct data_frame *frame);
static bool recv_current_pose_frame(struct data_frame *frame);
static bool recv_path_frame(struct data_frame *frame);
static bool recv_motor_frame(struct data_frame *frame);

#define MAX_BORDER_POINTS 100
static struct point2d data_border_points[MAX_BORDER_POINTS];

#define MAX_OBSTACLE_POINTS 100
static struct point2d data_obstacle_points[MAX_OBSTACLE_POINTS];

#define MAX_PATH_POINTS 9600
static struct path_point data_path_points[MAX_PATH_POINTS];

void data_recv_frame(struct data_frame *frame)
{
    while (1) {
        switch (recv_frame_header()) {
            case DATA_TYPE_MOVE:
                if (recv_move_frame(frame)) return;
                break;
            case DATA_TYPE_SCAN:
                if (recv_scan_frame(frame)) return;
                break;
            case DATA_TYPE_ESTIMATED_POSE:
                if (recv_estimated_pose_frame(frame)) return;
                break;
            case DATA_TYPE_CURRENT_POSE:
                if (recv_current_pose_frame(frame)) return;
                break;
            case DATA_TYPE_PATH:
                if (recv_path_frame(frame)) return;
                break;
            case DATA_TYPE_MOTOR:
                if (recv_motor_frame(frame)) return;
                break;
            default:
                continue;
        }
    }
}

static bool recv_move_frame(struct data_frame *frame)
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

    frame->type = DATA_TYPE_MOVE;
    frame->move.t = t;
    frame->move.delta_x = delta_x;
    frame->move.delta_y = delta_y;
    frame->move.delta_theta = delta_theta;
    return true;
}

static bool recv_scan_frame(struct data_frame *frame)
{
    uint32_t t;
    uint16_t border_point_count;
    uint16_t obstacle_point_count;
    struct point2d *border_points = NULL;
    struct point2d *obstacle_points = NULL;

    if (!recv_u32(&t)) goto err;

    if (!recv_u16(&border_point_count)) goto err;
    border_points = data_acquire_border_points();
    if (!recv_buf((uint8_t *) border_points, 2 * border_point_count)) goto err;

    if (!recv_u16(&obstacle_point_count)) goto err;
    obstacle_points = data_acquire_obstacle_points();
    if (!recv_buf((uint8_t *) obstacle_points, 2 * obstacle_point_count)) goto err;

    if (!recv_frame_checksum()) goto err;

    frame->type = DATA_TYPE_SCAN;
    frame->scan.t = t;
    frame->scan.border_point_count = border_point_count;
    frame->scan.border_points = border_points;
    frame->scan.obstacle_point_count = obstacle_point_count;
    frame->scan.obstacle_points = obstacle_points;
    return true;

err:
    if (border_points != NULL) {
        data_release_border_points();
    }
    if (obstacle_points != NULL) {
        data_release_obstacle_points();
    }
    return false;
}

static bool recv_estimated_pose_frame(struct data_frame *frame)
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

    frame->type = DATA_TYPE_ESTIMATED_POSE;
    frame->estimated_pose.t = t;
    frame->estimated_pose.x = x;
    frame->estimated_pose.y = y;
    frame->estimated_pose.theta = theta;
    return true;
}

static bool recv_current_pose_frame(struct data_frame *frame)
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

    frame->type = DATA_TYPE_CURRENT_POSE;
    frame->current_pose.t = t;
    frame->current_pose.x = x;
    frame->current_pose.y = y;
    frame->current_pose.theta = theta;
    return true;
}

static bool recv_path_frame(struct data_frame *frame)
{
    uint32_t t;
    uint16_t point_count;
    struct path_point *points = NULL;

    if (!recv_u32(&t)) goto err;

    if (!recv_u16(&point_count)) goto err;
    points = data_acquire_path_points();
    if (!recv_buf((uint8_t *) points, 2 * point_count)) goto err;

    if (!recv_frame_checksum()) goto err;

    frame->type = DATA_TYPE_PATH;
    frame->path.t = t;
    frame->path.point_count = point_count;
    frame->path.points = points;
    return true;

err:
    if (points != NULL) {
        data_release_path_points();
    }
    return false;
}

static bool recv_motor_frame(struct data_frame *frame)
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

    frame->type = DATA_TYPE_MOTOR;
    frame->motor.t = t;
    frame->motor.speed_a = speed_a;
    frame->motor.speed_b = speed_b;
    frame->motor.speed_c = speed_c;
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

struct point2d *data_acquire_border_points()
{
    // TODO: acquire semaphore
    return data_border_points;
}

void data_release_border_points()
{
    // TODO: release semaphore
}

struct point2d *data_acquire_obstacle_points()
{
    // TODO: acquire semaphore
    return data_obstacle_points;
}

void data_release_obstacle_points()
{
    // TODO: release semaphore
}

struct path_point *data_acquire_path_points()
{
    // TODO: acquire semaphore
    return data_path_points;
}

void data_release_path_points()
{
    // TODO: release semaphore
}