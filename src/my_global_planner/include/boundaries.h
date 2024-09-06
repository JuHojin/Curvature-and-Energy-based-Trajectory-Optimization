// boundaries.h
#ifndef BOUNDARIES_H
#define BOUNDARIES_H

#include <vector>
#include <Eigen/Dense>

namespace my_global_planner {

struct Boundary {
    std::vector<Eigen::Vector2d> points;
    std::vector<double> distances;
};

struct Boundaries {
    Boundary left_bnd;
    Boundary right_bnd;
};

} // namspace my_global_planner

#endif // BOUNDARIES_H