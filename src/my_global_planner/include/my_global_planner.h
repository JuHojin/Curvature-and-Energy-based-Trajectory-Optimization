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

namespace my_global_planner {

class MyGlobalPlanner : public nav_core::BaseGlobalPlanner {
public:
    // Constructors
    MyGlobalPlanner();
    MyGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    ~MyGlobalPlanner();

    // Overridden methods from nav_core::BaseGlobalPlanner
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);

private:
    struct Boundary {
        std::vector<geometry_msgs::PoseStamped> points;
        std::vector<float> distances;
    };
    struct Boundaries {
        Boundary left_bnd;
        Boundary right_bnd;
    };

    costmap_2d::Costmap2D* costmap;
    basic_planner::AStarPlanner* a_star_planner;

    // Publisher
    ros::Publisher plan_publisher_a_star;
    ros::Publisher plan_publisher_pose_smooth;
    ros::Publisher plan_publisher_uniform;
    ros::Publisher plan_publisher_bnd_L;
    ros::Publisher plan_publisher_bnd_R;

    void updateOrientatoin(std::vector<geometry_msgs::PoseStamped>& plan);
    void publishPath(const ros::Publisher& publisher, nav_msgs::Path& path_msg, const std::string& frame_id);
    nav_msgs::Path producePathMapToWorld (std::vector<geometry_msgs::PoseStamped>& plan);
    bool isClearLine (geometry_msgs::PoseStamped& start_pose, geometry_msgs::PoseStamped& end_pose);
    std::vector<geometry_msgs::PoseStamped> postSmooth (std::vector<geometry_msgs::PoseStamped>& plan);

    // Boundary
    void getBoundary (Boundaries& boundaries, const std::vector<geometry_msgs::PoseStamped>& plan);
    void constrainDeviationChange(std::vector<float>& dev_list);
    double checkBoundary(const geometry_msgs::PoseStamped& pose, double dx, double dy);
    geometry_msgs::PoseStamped createBoundaryPoint(const geometry_msgs::PoseStamped& pose, double dx, double dy, double distance);

    // Spacing
    std::vector<geometry_msgs::PoseStamped> uniformSpacing(const std::vector<geometry_msgs::PoseStamped>& plan);
    std::vector<geometry_msgs::PoseStamped> adaptiveSpacing(const std::vector<geometry_msgs::PoseStamped>& plan);
};

} // namespace my_global_planner

#endif // MY_GLOBAL_PLANNER_H