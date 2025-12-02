// path_func.h
#ifndef PATH_FUNC_H
#define PATH_FUNC_H

#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>

namespace my_global_planner {

// Utilities
void updateOrientation(std::vector<geometry_msgs::PoseStamped>& plan);
bool isClearLine(geometry_msgs::PoseStamped& start_pose, geometry_msgs::PoseStamped& end_pose);
bool isClearLine(int start_x, int start_y, int end_x, int end_y);
double calculateDistance(const Eigen::Vector2d& pt1, const Eigen::Vector2d& pt2);
Eigen::Vector2d getCircumcenter(const Eigen::Vector2d& pt1, const Eigen::Vector2d& pt2, const Eigen::Vector2d& pt3);
double getTurningRadius(const Eigen::Vector2d& pt1, const Eigen::Vector2d& pt2, const Eigen::Vector2d& pt3);
std::tuple<Eigen::VectorXd, double, double> getCurvatureSquareMaxUsingTurningRadius(const std::vector<Eigen::Vector2d>& path);
Eigen::VectorXd getCurvatureUsingTurningRadius(const std::vector<Eigen::Vector2d>& path);
Eigen::VectorXd getCurvatureUsingTurningRadius(const nav_msgs::Path& path);
Eigen::VectorXd getCurvatureUsingTurningRadius(const std::vector<geometry_msgs::PoseStamped>& plan);
Eigen::VectorXd getCurvatureUsingFiniteDifference(const std::vector<Eigen::Vector2d>& path);
Eigen::VectorXd getCurvatureUsingFiniteDifference(const nav_msgs::Path& path_msg);

// Post Smoothing
std::vector<geometry_msgs::PoseStamped> postSmooth(std::vector<geometry_msgs::PoseStamped>& plan);

// Spacing
std::vector<geometry_msgs::PoseStamped> uniformSpacing(const std::vector<geometry_msgs::PoseStamped>& plan, const double step_size);
std::vector<Eigen::Vector2d> uniformSpacing(const std::vector<Eigen::Vector2d>& path, const double step_size);
std::vector<Eigen::Vector2d> adaptiveSpacing(const std::vector<Eigen::Vector2d>& path, const double min_step_size, const double max_step_size);
std::vector<Eigen::Vector2d> fixedCountSpacing(const std::vector<Eigen::Vector2d>& path, std::size_t target_points);
double get_min_obstacle_distance(Eigen::Vector2d start, unsigned int searching_radius);

// Velocity profile
std::pair<Eigen::VectorXd, Eigen::MatrixXd> getVelocityProfile(const std::vector<Eigen::Vector2d> path, const Eigen::VectorXd& curvature, double init_velocity, double max_vel, double max_ax, double max_ay);
std::pair<Eigen::VectorXd, Eigen::MatrixXd> getVelocityProfile(const nav_msgs::Path& path_msg, const Eigen::VectorXd& curvature, double init_velocity, double max_vel, double max_ax, double max_ay);
void showVelocityProfile(const Eigen::VectorXd& velocity_profile, const Eigen::MatrixXd& velocity_constraints);

// Spline
bool callBSplineSmoothingService(const std::vector<geometry_msgs::PoseStamped>& input_plan, std::vector<geometry_msgs::PoseStamped>& output_plan, double smoothing_factor=-1, int spline_degree=-1, int num_splice_points=-1);

} // namespace my_global_planner

#endif // PATH_FUNC_H
