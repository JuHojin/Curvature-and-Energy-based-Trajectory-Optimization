// constant.h
#ifndef CONSTANT_H
#define CONSTANT_H

namespace Constant {
    // Boundary
    constexpr unsigned int DEV_MAX = 10;
    constexpr unsigned int DEV_DOT = 1;

    // Spacing
    constexpr unsigned int UNIFORM_STEP = 10;
    constexpr unsigned int ADAPTIVE_MIN_STEP = 5;
    constexpr unsigned int ADAPTIVE_MAX_STEP = 13;

    // Minimum curvature optimization
    constexpr double CURVATURE_BOUND = 0.7;
    constexpr unsigned int MAX_OPT_ITER = 10;

    // Velocity Profile
    constexpr double MAX_VEL = 20;
    constexpr double MAX_AX = 1;
    constexpr double MAX_AY = 0.5;
    constexpr double INIT_VELOCITY = 0;

    // Map: Obstacle
    constexpr unsigned char OBSTACLE_INFLATION_INIT = 150;
    constexpr unsigned char OBSTACLE_INFLATION = 170;
}

#endif // CONSTANT_H