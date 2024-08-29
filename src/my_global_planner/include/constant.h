// constant.h
#ifndef CONSTANT_H
#define CONSTANT_H

namespace Constant {
    // Boundary
    constexpr unsigned int DEV_MAX = 10;
    constexpr unsigned int DEV_DOT = 1;

    // Spacing
    constexpr unsigned int UNIFORM_STEP = 8;

    // Minimum curvature optimization
    constexpr double CURVATURE_BOUND = 0.7;
    constexpr unsigned int MAX_OPT_ITER = 10;

    // Map: Obstacle
    constexpr unsigned char OBSTACLE_INFLATION_INIT = 160;
    constexpr unsigned char OBSTACLE_INFLATION = 200;
}

#endif // CONSTANT_H