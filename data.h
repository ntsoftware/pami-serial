#pragma once

#include <stddef.h>
#include <stdint.h>

struct vec2d {
    int16_t x; // mm
    int16_t y; // mm
};

struct point2d {
    uint16_t x; // mm
    uint16_t y; // mm
};

struct path_point {
    uint8_t ix;
    uint8_t iy;
};

enum robot_mode {
    ROBOT_MODE_DEBUG = 0,
    ROBOT_MODE_MATCH = 1,
    ROBOT_MODE_STOP = 2,
};

enum team_color {
    TEAM_COLOR_YELLOW = 0,
    TEAM_COLOR_BLUE = 1,
};

enum data_frame_type {
    DATA_TYPE_HEARTBEAT = 1,
    DATA_TYPE_MOVE = 2,
    DATA_TYPE_SCAN = 3,
    DATA_TYPE_ESTIMATED_POSE = 4,
    DATA_TYPE_CURRENT_POSE = 5,
    DATA_TYPE_PATH = 6,
    DATA_TYPE_MOTOR = 7,
};
