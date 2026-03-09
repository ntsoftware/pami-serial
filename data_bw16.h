#pragma once

#include "data.h"

struct data_move {
    uint32_t t;
    int16_t delta_x; // mm
    int16_t delta_y; // mm
    int16_t delta_theta; // deg
};

struct data_scan {
    uint32_t t;
    uint16_t border_point_count;
    const struct point2d *border_points;
    uint16_t obstacle_point_count;
    const struct point2d *obstacle_points;
};

struct data_estimated_pose {
    uint32_t t;
    uint16_t x; // mm
    uint16_t y; // mm
    int16_t theta; // deg
};

struct data_current_pose {
    uint32_t t;
    uint16_t x; // mm
    uint16_t y; // mm
    int16_t theta; // deg
};

struct data_path {
    uint32_t t;
    uint16_t point_count;
    const struct path_point *points;
};

struct data_motor {
    uint32_t t;
    int16_t speed_a; // mm/s
    int16_t speed_b; // mm/s
    int16_t speed_c; // mm/s
};

struct data_frame {
    enum data_frame_type type;
    union {
        struct data_move move;
        struct data_scan scan;
        struct data_estimated_pose estimated_pose;
        struct data_current_pose current_pose;
        struct data_path path;
        struct data_motor motor;
    };
};

// this function blocks until it receives a valid data frame
void data_recv_frame(struct data_frame *frame);

struct point2d *data_acquire_border_points();
void data_release_border_points();

struct point2d *data_acquire_obstacle_points();
void data_release_obstacle_points();

struct path_point *data_acquire_path_points();
void data_release_path_points();
