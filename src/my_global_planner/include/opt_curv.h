// opt_curv.h
#ifndef OPT_CURV_H
#define OPT_CURV_H

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <osqp.h>
#include <vector>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>

#include "boundaries.h"

namespace my_global_planner {

std::vector<Eigen::Vector2d> optimize_min_curvature(const std::vector<Eigen::Vector2d>& path, const size_t max_iter);

void updateSystemMatrix(const std::vector<Eigen::Vector2d>& path, Eigen::MatrixXd& A);
void updateNormalVectors(const std::vector<Eigen::Vector2d>& path, std::vector<Eigen::Vector2d>& normal_vectors);

Eigen::VectorXd getCurvatureUsingSpline(const std::vector<Eigen::Vector2d>& path, const Eigen::MatrixXd& A, double start_angle, double end_angle);
double getTurningRadius(const Eigen::Vector2d& pt1, const Eigen::Vector2d& pt2, const Eigen::Vector2d& pt3);
void getCurvatureUsingTurningRadius(const std::vector<Eigen::Vector2d>& path, std::vector<double>& curvature, double& curvature_square_sum, double& max_curvature);

Eigen::VectorXd getAlphaMinCurvature(const std::vector<Eigen::Vector2d>& path,
                                     std::vector<double>& left_dev_bnd,
                                     std::vector<double>& right_dev_bnd,
                                     const std::vector<Eigen::Vector2d>& normal_vectors,
                                     const Eigen::MatrixXd& A,
                                     double curvature_bnd,
                                     double start_angle, double end_angle);

// Boundaries
void updateBoundaries(Boundaries& boundaries, const std::vector<Eigen::Vector2d>& path, const std::vector<Eigen::Vector2d>& normal_vectors);
void constrainDeviationChange(std::vector<double>& dev_list);
double checkBoundary(const Eigen::Vector2d& point, double dx, double dy);
Eigen::Vector2d createBoundaryPoint(const Eigen::Vector2d& point, double dx, double dy, double distance);

void setupOptimizer(OSQPSettings* settings);

} // namespace my_global_planner

#endif // OPT_CURV_H
