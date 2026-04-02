#pragma once

#include "data.h"
#include "hal.h"

class Capture {
public:
    Capture(const char *path);
    ~Capture();

    void write_frame(DataFrame &frame);

private:
    hal::File out;

    void write_header();
    void write_move_frame(DataFrame::Move &frame);
    void write_scan_frame(DataFrame::Scan &frame);
    void write_estimated_pose_frame(DataFrame::EstimatedPose &frame);
    void write_current_pose_frame(DataFrame::CurrentPose &frame);
    void write_path_frame(DataFrame::Path &frame);
    void write_motor_frame(DataFrame::Motor &frame);
};
