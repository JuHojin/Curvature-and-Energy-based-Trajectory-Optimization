// constant.h
#ifndef CONSTANT_H
#define CONSTANT_H

namespace Constant {
    // Map
    constexpr double MAP_RESOLUTION = 0.05;

    // Boundary
    constexpr unsigned int DEV_MAX = 15;
    constexpr unsigned int DEV_DOT = 1;
    constexpr unsigned int DEV_EPSILON = 1; // trimming parameter when boundaries are crossed

    // Spacing
    constexpr double UNIFORM_STEP = 5;
    constexpr double ADAPTIVE_MIN_STEP = 5;
    constexpr double ADAPTIVE_MAX_STEP = 10;
    constexpr double LAST_UNIFORM_STEP = 1;

    // Minimum curvature optimization
    // constexpr double CURVATURE_BOUND = 0.5 * MAP_RESOLUTION;
    constexpr double CURVATURE_BOUND_MAX = 1.5 * MAP_RESOLUTION;
    constexpr double CURVATURE_BOUND_MIN = 0.5 * MAP_RESOLUTION;
    constexpr double CURVATURE_BOUND_DECAY = 0.6;
    constexpr unsigned int MAX_OPT_ITER = 10;
    constexpr unsigned int MAX_OPT_ITER_REPLAN = 3;
    // 0 (QP), 1e-8 (QP_E): paper experiments were run with both values
    constexpr double GAMMA = 1e-8; 
    
    // Velocity Profile [SI]
    constexpr double MAX_VEL = 5;
    constexpr double MAX_AX = 1;
    constexpr double MAX_AY = 2;
    constexpr double INIT_VELOCITY = 0;

    // Pin point
    constexpr int NEIGHBOR_RANGE = 30;
    constexpr int PINPOINT_SELECT_METHOD = 3;

    // Map: Obstacle
    constexpr unsigned char OBSTACLE_INFLATION_INIT = 120;
    constexpr unsigned char OBSTACLE_INFLATION = 120;
    constexpr unsigned char LOCAL_OBSTACLE_INFLATION = 10;

    // Spline: B-spline
    constexpr double SMOOTHING_FACTOR = 0.1;
    constexpr int SPLINE_DEGREE = 3;
    constexpr int NUM_SPLINE_POINTS = 10;
}

#endif // CONSTANT_H