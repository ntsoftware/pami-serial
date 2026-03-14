#include "data.h"

static struct point2d data_border_points[DATA_MAX_BORDER_POINTS];
static struct point2d data_obstacle_points[DATA_MAX_OBSTACLE_POINTS];
static struct path_point data_path_points[DATA_MAX_PATH_POINTS];

struct point2d *data_acquire_border_points()
{
    // TODO: acquire semaphore
    return data_border_points;
}

void data_release_border_points()
{
    // TODO: release semaphore
}

struct point2d *data_acquire_obstacle_points()
{
    // TODO: acquire semaphore
    return data_obstacle_points;
}

void data_release_obstacle_points()
{
    // TODO: release semaphore
}

struct path_point *data_acquire_path_points()
{
    // TODO: acquire semaphore
    return data_path_points;
}

void data_release_path_points()
{
    // TODO: release semaphore
}