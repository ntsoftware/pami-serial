#pragma once

#include <stddef.h>
#include <stdint.h>

struct Vec2d {
    int16_t x; // mm
    int16_t y; // mm
};

struct Point2d {
    uint16_t x; // mm
    uint16_t y; // mm
};

struct PathPoint {
    uint8_t ix;
    uint8_t iy;
};

enum RobotMode {
    ROBOT_MODE_DEBUG = 0,
    ROBOT_MODE_MATCH = 1,
    ROBOT_MODE_STOP = 2,
};

enum TeamColor {
    TEAM_COLOR_YELLOW = 0,
    TEAM_COLOR_BLUE = 1,
};

enum FrameType {
    FRAME_TYPE_INVALID = 0,
    FRAME_TYPE_HEARTBEAT = 1,
    FRAME_TYPE_MOVE = 2,
    FRAME_TYPE_SCAN = 3,
    FRAME_TYPE_ESTIMATED_POSE = 4,
    FRAME_TYPE_CURRENT_POSE = 5,
    FRAME_TYPE_PATH = 6,
    FRAME_TYPE_MOTOR = 7,
};

struct Heartbeat {
    RobotMode robot_mode;
    TeamColor team_color;
    uint8_t goal_zone;
    uint32_t game_time;
};
