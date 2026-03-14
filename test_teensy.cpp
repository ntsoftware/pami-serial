#include <stdio.h>
#include "teensy/data.h"
#include "teensy/hal.h"

int main()
{
    hal_init();
    data_init();

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

    while (1) {
        struct data_heartbeat heartbeat;

        if (data_recv_heartbeat_frame(&heartbeat)) {
            printf("heartbeat frame\n");
            printf("robot_mode=%d\n", heartbeat.robot_mode);
            printf("team_color=%d\n", heartbeat.team_color);
            printf("goal_zone=%d\n", heartbeat.goal_zone);
            printf("game_time=%d\n", heartbeat.game_time);
        }
    }

    return 0;
}