// path_func.h
#ifndef PATH_FUNC_H
#define PATH_FUNC_H

#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>

namespace my_global_planner {

// Utilities
void updateOrientatoin(std::vector<geometry_msgs::PoseStamped>& plan);
bool isClearLine(geometry_msgs::PoseStamped& start_pose, geometry_msgs::PoseStamped& end_pose);
bool isClearLine(int start_x, int start_y, int end_x, int end_y);
double getTurningRadius(const Eigen::Vector2d& pt1, const Eigen::Vector2d& pt2, const Eigen::Vector2d& pt3);
void getCurvatureUsingTurningRadius(const std::vector<Eigen::Vector2d>& path, std::vector<double>& curvature, double& curvature_square_sum, double& max_curvature);

// Post Smoothing
std::vector<geometry_msgs::PoseStamped> postSmooth(std::vector<geometry_msgs::PoseStamped>& plan);

// Spacing
std::vector<geometry_msgs::PoseStamped> uniformSpacing(const std::vector<geometry_msgs::PoseStamped>& plan);
std::vector<Eigen::Vector2d> uniformSpacing(const std::vector<Eigen::Vector2d>& path);
std::vector<geometry_msgs::PoseStamped> adaptiveSpacing(const std::vector<geometry_msgs::PoseStamped>& plan);

// Velocity profile
std::pair<Eigen::VectorXd, Eigen::MatrixXd> getVelocityProfile(const Eigen::VectorXd& curvature, double init_velocity, double max_vel, double max_ax, double max_ay);
void showVelocityProfile(const Eigen::VectorXd& velocity_profile, const Eigen::MatrixXd& velocity_constraints);

} // namespace my_global_planner

#endif // PATH_FUNC_H