// path_func.h
#ifndef PATH_FUNC_H
#define PATH_FUNC_H

#include <vector>
#include <geometry_msgs/PoseStamped.h>

namespace my_global_planner {

// Utilities
void updateOrientatoin(std::vector<geometry_msgs::PoseStamped>& plan);
bool isClearLine(geometry_msgs::PoseStamped& start_pose, geometry_msgs::PoseStamped& end_pose);
bool isClearLine(int start_x, int start_y, int end_x, int end_y);

// Post Smoothing
std::vector<geometry_msgs::PoseStamped> postSmooth(std::vector<geometry_msgs::PoseStamped>& plan);

// Spacing
std::vector<geometry_msgs::PoseStamped> uniformSpacing(const std::vector<geometry_msgs::PoseStamped>& plan);
std::vector<Eigen::Vector2d> uniformSpacingVector2d(const std::vector<Eigen::Vector2d>& path);
std::vector<geometry_msgs::PoseStamped> adaptiveSpacing(const std::vector<geometry_msgs::PoseStamped>& plan);

} // namespace my_global_planner

#endif // PATH_FUNC_H