// my_global_planner.h
#ifndef MY_GLOBAL_PLANNER_H
#define MY_GLOBAL_PLANNER_H

#include "a_star.h"

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <vector>
#include <Eigen/Dense>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseArray.h>

namespace my_global_planner {

// Costmap
extern costmap_2d::Costmap2D* costmap;

// Publisher
extern ros::Publisher plan_publisher_bnd_L;
extern ros::Publisher plan_publisher_bnd_R;
extern ros::Publisher plan_publisher_opt;

void publishPath(const ros::Publisher& publisher, nav_msgs::Path& path_msg);
void publishVelocityProfile(const ros::Publisher& profile_pub, const Eigen::VectorXd& velocity_profile);
void publishVelocityConstraints(const ros::Publisher& constraints_pub, const Eigen::MatrixXd& velocity_constraints);

std::pair<std::vector<int>, geometry_msgs::PoseArray> pin_point_select(const std::vector<Eigen::Vector2d>& path);
std::pair<geometry_msgs::PoseArray, std::vector<int>> updatePinPoints(const geometry_msgs::PoseArray& opt_pin_point_list, const std::vector<geometry_msgs::PoseStamped>& final_plan);

nav_msgs::Path producePathMapToWorld (std::vector<geometry_msgs::PoseStamped>& plan);
std::vector<Eigen::Vector2d> convertPoseStampedToVector2d(const std::vector<geometry_msgs::PoseStamped>& plan);
std::vector<geometry_msgs::PoseStamped> convertVector2dToPoseStamped(const std::vector<Eigen::Vector2d>& plan);
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

private:
    bool replanning_mode;

    ros::Publisher plan_publisher_a_star;
    ros::Publisher plan_publisher_pose_smooth;
    ros::Publisher plan_publisher_uniform;
    ros::Publisher plan_publisher_final;
    ros::Publisher plan_publisher_last_opt;
    ros::Publisher profile_publisher;
    ros::Publisher constraints_publisher;
    ros::Publisher pin_points_publisher;
    ros::Publisher pin_point_indices_publisher;


    ros::Subscriber replanning_trigger_sub;
    
    void replanningTriggerCallback(const std_msgs::Bool::ConstPtr& msg);
};

} // namespace my_global_planner

#endif // MY_GLOBAL_PLANNER_H