#pragma once

#include "../data_types.h"

void data_init();

// this function returns true if a valid heartbeat frame is received in the out buffer
bool data_recv_heartbeat_frame(struct data_heartbeat *out);

void data_send_move_frame(int16_t delta_x, int16_t delta_y, int16_t delta_theta);

void data_send_scan_frame(
    const struct point2d *border_points, size_t border_point_count,
    const struct point2d *obstacle_points, size_t obstacle_point_count
);

void data_send_estimated_pose_frame(uint16_t x, uint16_t y, int16_t theta);
void data_send_current_pose_frame(uint16_t x, uint16_t y, int16_t theta);

void data_send_path_frame(const struct path_point *points, size_t point_count);

void data_send_motor_frame(int16_t speed_a, int16_t speed_b, int16_t speed_c);
