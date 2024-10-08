// my_global_planner.h
#ifndef MY_GLOBAL_PLANNER_H
#define MY_GLOBAL_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <angles/angles.h>
#include <vector>
#include <Eigen/Dense>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "my_global_planner/ReplanningService.h"
#include "a_star.h"

namespace my_global_planner {

// Costmap
extern costmap_2d::Costmap2D* costmap;
extern nav_msgs::OccupancyGrid::ConstPtr local_costmap_msg;

// Publisher
extern ros::Publisher plan_publisher_bnd_L;
extern ros::Publisher plan_publisher_bnd_R;
extern ros::Publisher plan_publisher_bnd_L_replan;
extern ros::Publisher plan_publisher_bnd_R_replan;
extern ros::Publisher plan_publisher_opt;

void publishPath(const ros::Publisher& publisher, nav_msgs::Path& path_msg);
void publishVelocityProfile(const ros::Publisher& profile_pub, const Eigen::VectorXd& velocity_profile);
void publishVelocityConstraints(const ros::Publisher& constraints_pub, const Eigen::MatrixXd& velocity_constraints);

std::pair<std::vector<int>, geometry_msgs::PoseArray> pin_point_select(const std::vector<Eigen::Vector2d>& path);
std::pair<geometry_msgs::PoseArray, std::vector<int>> updatePinPoints(const geometry_msgs::PoseArray& opt_pin_point_list, const std::vector<geometry_msgs::PoseStamped>& final_plan);

nav_msgs::Path producePathMapToWorld (std::vector<geometry_msgs::PoseStamped>& plan);
std::vector<Eigen::Vector2d> convertPoseStampedToVector2d(const std::vector<geometry_msgs::PoseStamped>& plan);
std::vector<geometry_msgs::PoseStamped> convertVector2dToPoseStamped(const std::vector<Eigen::Vector2d>& plan);
bool convertMapToWorld(const unsigned int& mx, const unsigned int& my, double& wx, double& wy);
bool convertMapToWorld(const double& mx, const double& my, double& wx, double& wy);
bool convertWorldToMap(const double& wx, const double& wy, unsigned int &mx, unsigned int &my);
bool convertWorldToMap(const double& wx, const double& wy, double &mx, double &my);

geometry_msgs::PoseStamped localToGlobal(const Eigen::Vector2d& local_point, const nav_msgs::OccupancyGrid& local_costmap_msg, const std::string& target_frame, tf2_ros::Buffer& tf_buffer);
Eigen::Vector2d globalToLocal(const Eigen::Vector2d& global_point, const nav_msgs::OccupancyGrid& local_costmap_msg, const std::string& target_frame, tf2_ros::Buffer& tf_buffer);

class MyGlobalPlanner : public nav_core::BaseGlobalPlanner {
public:
    // Constructors
    MyGlobalPlanner();
    MyGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    // Destructor
    ~MyGlobalPlanner();

    // Overridden methods from nav_core::BaseGlobalPlanner
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);
    void replanningIndicesCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);
    bool handleReplanningRequest(my_global_planner::ReplanningService::Request &req, my_global_planner::ReplanningService::Response &res);

private:
    geometry_msgs::PoseStamped start_;
    geometry_msgs::PoseStamped goal_;
    std::vector<geometry_msgs::PoseStamped> latest_plan_global;
    bool replanning_mode;
    std::array<int, 5> received_pin_point_indices_;
    nav_msgs::Path replanned_path_world;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Publisher plan_publisher_a_star;
    ros::Publisher plan_publisher_pose_smooth;
    ros::Publisher plan_publisher_uniform;
    ros::Publisher plan_publisher_initial;
    ros::Publisher plan_publisher_replan;
    ros::Publisher plan_publisher_last_opt;
    ros::Publisher plan_publisher_latest;
    ros::Publisher profile_publisher;
    ros::Publisher constraints_publisher;
    ros::Publisher pin_points_publisher;
    ros::Publisher pin_point_indices_publisher;
    ros::Publisher replan_status_publisher;
    ros::Publisher planning_calculation_time_publisher;
    ros::Publisher curvature_publisher;

    ros::Subscriber replanning_trigger_sub;
    ros::Subscriber replanning_indices_sub;
    ros::Subscriber local_costmap_sub;

    ros::ServiceServer replan_service_;
    
    void replanningTriggerCallback(const std_msgs::Bool::ConstPtr& msg);
    void localCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
};

} // namespace my_global_planner

#endif // MY_GLOBAL_PLANNER_H