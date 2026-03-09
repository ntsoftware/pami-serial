#include <stdio.h>
#include "data.h"
#include "data_bw16.h"
#include "data_bw16_hal.h"

static void print_points(const struct point2d *points, size_t n)
{
    putchar('[');
    for (size_t i = 0; i < n; ++i) {
        printf("(x=%d,y=%d)", points[i].x, points[i].y);
        if (i != n - 1)
            putchar(',');
    }
    putchar(']');
}

static void print_path(const struct path_point *points, size_t n)
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
    data_hal_init();

    while (1) {
        struct data_frame frame;
        data_recv_frame(&frame);

        switch (frame.type) {
            case DATA_TYPE_MOVE:
                printf("move frame\n");
                printf("t=%d\n", frame.move.t);
                printf("delta_x=%d\n", frame.move.delta_x);
                printf("delta_y=%d\n", frame.move.delta_y);
                printf("delta_theta=%d\n", frame.move.delta_theta);
                break;
            case DATA_TYPE_SCAN:
                printf("scan frame\n");
                printf("t=%d\n", frame.scan.t);
                printf("border_points=");
                print_points(frame.scan.border_points, frame.scan.border_point_count);
                putchar('\n');
                printf("obstacle_points=");
                print_points(frame.scan.obstacle_points, frame.scan.obstacle_point_count);
                putchar('\n');
                data_release_border_points();
                data_release_obstacle_points();
                break;
            case DATA_TYPE_ESTIMATED_POSE:
                printf("estimated pose frame\n");
                printf("t=%d\n", frame.estimated_pose.t);
                printf("x=%d\n", frame.estimated_pose.x);
                printf("y=%d\n", frame.estimated_pose.y);
                printf("theta=%d\n", frame.estimated_pose.theta);
                break;
            case DATA_TYPE_CURRENT_POSE:
                printf("current pose frame\n");
                printf("t=%d\n", frame.current_pose.t);
                printf("x=%d\n", frame.current_pose.x);
                printf("y=%d\n", frame.current_pose.y);
                printf("theta=%d\n", frame.current_pose.theta);
                break;
            case DATA_TYPE_PATH:
                printf("path frame\n");
                printf("t=%d\n", frame.estimated_pose.t);
                printf("path_points=");
                print_path(frame.path.points, frame.path.point_count);
                putchar('\n');
                data_release_path_points();
                break;
            case DATA_TYPE_MOTOR:
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