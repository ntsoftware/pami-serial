#include <string.h>
#include "capture.h"

Capture::Capture(const char *path)
{
    // TODO: find next available file name
    out.open(path);
    write_header();
}

Capture::~Capture()
{
    out.close();
}

void Capture::write_frame(DataFrame &frame)
{
    switch (frame.type) {
        case FRAME_TYPE_MOVE:
            write_move_frame(frame.move);
            break;
        case FRAME_TYPE_SCAN:
            write_scan_frame(frame.scan);
            break;
        case FRAME_TYPE_ESTIMATED_POSE:
            write_estimated_pose_frame(frame.estimated_pose);
            break;
        case FRAME_TYPE_CURRENT_POSE:
            write_current_pose_frame(frame.current_pose);
            break;
        case FRAME_TYPE_PATH:
            write_path_frame(frame.path);
            break;
        case FRAME_TYPE_MOTOR:
            write_motor_frame(frame.motor);
            break;
        default:
            break;
    }
}

void Capture::write_header()
{
    static const uint8_t buf[4] = {0x01, 0x00, 0x00, 0x00};
    out.write(buf, sizeof(buf));
}

void Capture::write_move_frame(DataFrame::Move &frame)
{
    uint8_t buf[11];
    buf[0] = FRAME_TYPE_MOVE;
    memcpy(&buf[1], &frame.t, 4);
    memcpy(&buf[5], &frame.delta_x, 2);
    memcpy(&buf[7], &frame.delta_y, 2);
    memcpy(&buf[9], &frame.delta_theta, 2);
    out.write(buf, sizeof(buf));
}

void Capture::write_scan_frame(DataFrame::Scan &frame)
{
    uint8_t buf[9];
    buf[0] = FRAME_TYPE_SCAN;
    memcpy(&buf[1], &frame.t, 4);
    memcpy(&buf[5], &frame.border_point_count, 2);
    memcpy(&buf[7], &frame.obstacle_point_count, 2);
    out.write(buf, sizeof(buf));
    out.write((const uint8_t *) frame.border_points.ptr(), frame.border_point_count * 4);
    out.write((const uint8_t *) frame.obstacle_points.ptr(), frame.obstacle_point_count * 4);
}

void Capture::write_estimated_pose_frame(DataFrame::EstimatedPose &frame)
{
    uint8_t buf[11];
    buf[0] = FRAME_TYPE_ESTIMATED_POSE;
    memcpy(&buf[1], &frame.t, 4);
    memcpy(&buf[5], &frame.x, 2);
    memcpy(&buf[7], &frame.y, 2);
    memcpy(&buf[9], &frame.theta, 2);
    out.write(buf, sizeof(buf));
}

void Capture::write_current_pose_frame(DataFrame::CurrentPose &frame)
{
    uint8_t buf[11];
    buf[0] = FRAME_TYPE_CURRENT_POSE;
    memcpy(&buf[1], &frame.t, 4);
    memcpy(&buf[5], &frame.x, 2);
    memcpy(&buf[7], &frame.y, 2);
    memcpy(&buf[9], &frame.theta, 2);
    out.write(buf, sizeof(buf));
}

void Capture::write_path_frame(DataFrame::Path &frame)
{
    uint8_t buf[7];
    buf[0] = FRAME_TYPE_PATH;
    memcpy(&buf[1], &frame.t, 4);
    memcpy(&buf[5], &frame.point_count, 2);
    out.write(buf, sizeof(buf));
    out.write((const uint8_t *) frame.points.ptr(), frame.point_count * 2);
}

void Capture::write_motor_frame(DataFrame::Motor &frame)
{
    uint8_t buf[11];
    buf[0] = FRAME_TYPE_MOTOR;
    memcpy(&buf[1], &frame.t, 4);
    memcpy(&buf[5], &frame.speed_a, 2);
    memcpy(&buf[7], &frame.speed_b, 2);
    memcpy(&buf[9], &frame.speed_c, 2);
    out.write(buf, sizeof(buf));
}
