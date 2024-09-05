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

std::vector<Eigen::Vector2d> optimize_min_curvature(const std::vector<Eigen::Vector2d> path, const size_t max_iter, bool local);

Eigen::MatrixXd getSystemMatrix(const std::vector<Eigen::Vector2d>& path);
std::vector<Eigen::Vector2d> getNormalVectors(const std::vector<Eigen::Vector2d>& path);

Eigen::VectorXd getCurvatureUsingSpline(const std::vector<Eigen::Vector2d>& path, const Eigen::MatrixXd& A, double start_angle, double end_angle);
double getTravelTime(const std::vector<Eigen::Vector2d>& path, const Eigen::VectorXd& velocity_profile);

// Boundaries
Boundaries getBoundaries(const std::vector<Eigen::Vector2d>& path, const std::vector<Eigen::Vector2d>& normal_vectors, bool use_local_costmap);
void constrainDeviationChange(std::vector<double>& dev_list);
double checkBoundary(const Eigen::Vector2d& point, double dx, double dy, bool local);
Eigen::Vector2d createBoundaryPoint(const Eigen::Vector2d& point, double dx, double dy, double distance);

csc* eigenToCSC(const Eigen::SparseMatrix<double>& mat);
std::pair<Eigen::VectorXd, int> getAlphaMinCurvature(const std::vector<Eigen::Vector2d>& path,
                                     std::vector<double> left_dev_bnd,
                                     std::vector<double> right_dev_bnd,
                                     const std::vector<Eigen::Vector2d>& normal_vectors,
                                     const Eigen::MatrixXd& A,
                                     double curvature_bnd,
                                     double start_angle, double end_angle);
std::vector<Eigen::Vector2d> getPathMinCurvature(const std::vector<Eigen::Vector2d>& path,
                                                 const std::vector<Eigen::Vector2d>& normal_vectors,
                                                 const Eigen::VectorXd& alpha);

} // namespace my_global_planner

#endif // OPT_CURV_H
