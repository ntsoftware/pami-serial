#include <stdio.h>
#include "data.h"
#include "data_teensy.h"
#include "data_teensy_hal.h"

int main()
{
    data_hal_init();

    struct data_heartbeat heartbeat;
    data_recv_heartbeat_frame(&heartbeat);
    printf("robot_mode=%d\n", heartbeat.robot_mode);
    printf("team_color=%d\n", heartbeat.team_color);
    printf("goal_zone=%d\n", heartbeat.goal_zone);
    printf("game_time=%d\n", heartbeat.game_time);

    data_send_move_frame(10, -10, 5);

    static const struct point2d border_points[] = {
        {.x = 1, .y = 2},
        {.x = 3, .y = 4},
        {.x = 5, .y = 6},
    };

    static const struct point2d obstacle_points[] = {
        {.x = 11, .y = 12},
        {.x = 13, .y = 14},
        {.x = 15, .y = 16},
    };

    data_send_scan_frame(border_points, 3, obstacle_points, 3);

    data_send_estimated_pose_frame(100, 200, 300);
    data_send_current_pose_frame(100, 200, 300);

    static const struct path_point path_points[] = {
        {.ix = 1, .iy = 2},
        {.ix = 3, .iy = 4},
        {.ix = 5, .iy = 6},
    };

    data_send_path_frame(path_points, 3);

    data_send_motor_frame(100, 200, 300);

    return 0;
}