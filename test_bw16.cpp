#include <stdio.h>
#include "bw16/data.h"

static void print_points(const Point2d *points, size_t n)
{
    putchar('[');
    for (size_t i = 0; i < n; ++i) {
        printf("(x=%d,y=%d)", points[i].x, points[i].y);
        if (i != n - 1)
            putchar(',');
    }
    putchar(']');
}

static void print_path(const PathPoint *points, size_t n)
{
    putchar('[');
    for (size_t i = 0; i < n; ++i) {
        printf("(ix=%d,iy=%d)", points[i].ix, points[i].iy);
        if (i != n - 1)
            putchar(',');
    }
    putchar(']');
}

int main()
{
    Data data;

    const Heartbeat heartbeat1 = {
        .robot_mode = ROBOT_MODE_DEBUG,
        .team_color = TEAM_COLOR_BLUE,
        .goal_zone = 10,
        .game_time = 12345,
    };
    data.send_heartbeat(heartbeat1);

    const Heartbeat heartbeat2 = {
        .robot_mode = ROBOT_MODE_MATCH,
        .team_color = TEAM_COLOR_YELLOW,
        .goal_zone = 20,
        .game_time = 67890,
    };
    data.send_heartbeat(heartbeat2);

    while (1) {
        DataFrame frame;
        data.recv_frame(frame);

        switch (frame.type) {
            case FRAME_TYPE_MOVE:
                printf("move frame\n");
                printf("t=%d\n", frame.move.t);
                printf("delta_x=%d\n", frame.move.delta_x);
                printf("delta_y=%d\n", frame.move.delta_y);
                printf("delta_theta=%d\n", frame.move.delta_theta);
                break;
            case FRAME_TYPE_SCAN:
                printf("scan frame\n");
                printf("t=%d\n", frame.scan.t);
                printf("border_points=");
                print_points(frame.scan.border_points.ptr(), frame.scan.border_point_count);
                putchar('\n');
                printf("obstacle_points=");
                print_points(frame.scan.obstacle_points.ptr(), frame.scan.obstacle_point_count);
                putchar('\n');
                frame.scan.border_points.release();
                frame.scan.obstacle_points.release();
                break;
            case FRAME_TYPE_ESTIMATED_POSE:
                printf("estimated pose frame\n");
                printf("t=%d\n", frame.estimated_pose.t);
                printf("x=%d\n", frame.estimated_pose.x);
                printf("y=%d\n", frame.estimated_pose.y);
                printf("theta=%d\n", frame.estimated_pose.theta);
                break;
            case FRAME_TYPE_CURRENT_POSE:
                printf("current pose frame\n");
                printf("t=%d\n", frame.current_pose.t);
                printf("x=%d\n", frame.current_pose.x);
                printf("y=%d\n", frame.current_pose.y);
                printf("theta=%d\n", frame.current_pose.theta);
                break;
            case FRAME_TYPE_PATH:
                printf("path frame\n");
                printf("t=%d\n", frame.estimated_pose.t);
                printf("path_points=");
                print_path(frame.path.points.ptr(), frame.path.point_count);
                putchar('\n');
                frame.path.points.release();
                break;
            case FRAME_TYPE_MOTOR:
                printf("motor frame\n");
                printf("t=%d\n", frame.motor.t);
                printf("speed_a=%d\n", frame.motor.speed_a);
                printf("speed_b=%d\n", frame.motor.speed_b);
                printf("speed_c=%d\n", frame.motor.speed_c);
                break;
            default:
                continue;
        }
    }
}
