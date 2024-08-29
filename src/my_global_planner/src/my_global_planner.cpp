// my_global_planner.cpp
#include "my_global_planner.h"
#include "a_star.h"
#include "path_func.h"
#include "opt_curv.h"
#include "constant.h"
#include "boundaries.h"

#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <cmath>
#include <std_msgs/Float64MultiArray.h>

// Register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(my_global_planner::MyGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace my_global_planner {

costmap_2d::Costmap2D* costmap = nullptr;
ros::Publisher plan_publisher_bnd_L;
ros::Publisher plan_publisher_bnd_R;
ros::Publisher plan_publisher_final;
ros::Publisher plan_publisher_opt;

MyGlobalPlanner::MyGlobalPlanner() {}

MyGlobalPlanner::MyGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros);
}

MyGlobalPlanner::~MyGlobalPlanner() {}

void MyGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    costmap = costmap_ros->getCostmap();
    ros::NodeHandle private_nh("~/" + name);
    plan_publisher_a_star = private_nh.advertise<nav_msgs::Path>("plan/a_star_plan", 1);
    plan_publisher_pose_smooth = private_nh.advertise<nav_msgs::Path>("plan/post_smooth", 1);
    plan_publisher_uniform = private_nh.advertise<nav_msgs::Path>("plan/uniform", 1);
    plan_publisher_bnd_L = private_nh.advertise<nav_msgs::Path>("plan/bnd_L", 1);
    plan_publisher_bnd_R = private_nh.advertise<nav_msgs::Path>("plan/bnd_R", 1);
    plan_publisher_opt = private_nh.advertise<nav_msgs::Path>("plan/opt", 1);
    profile_publisher = private_nh.advertise<std_msgs::Float64MultiArray>("velocity_profile", 10);
    constraints_publisher = private_nh.advertise<std_msgs::Float64MultiArray>("velocity_constraints", 10);
    plan_publisher_final = private_nh.advertise<nav_msgs::Path>("plan/final", 1);
    ROS_INFO("MyGlobalPlanner initialized successfully.");
}

bool MyGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                               const geometry_msgs::PoseStamped& goal,
                               std::vector<geometry_msgs::PoseStamped>& plan) {
                                
    plan.clear();

    // Convert start and goal point: (Real) World -> Map
    unsigned int start_x, start_y, goal_x, goal_y;
    if (!costmap->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
        !costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
        ROS_ERROR("Start or goal coordinates are out of bounds.");
        return false;
    }

    // Find initial Path: A* Planning
    ROS_INFO("Find path using A*.");
    basic_planner::AStarPlanner a_star_planner;
    std::vector<geometry_msgs::PoseStamped> a_star_plan;

    bool a_star_success = a_star_planner.makePlan(start_x, start_y, goal_x, goal_y, a_star_plan);
    updateOrientatoin(a_star_plan);
    nav_msgs::Path a_star_plan_world = producePathMapToWorld(a_star_plan);

    if (a_star_success) {
        ROS_INFO("Initial A* planning is successful.");

        publishPath(plan_publisher_a_star, a_star_plan_world);
        plan = a_star_plan_world.poses;
    } else {
        ROS_WARN("Initial A* planning failed.");
    }

    // Post smoothing
    std::vector<geometry_msgs::PoseStamped> post_smooth_plan = postSmooth(a_star_plan);
    nav_msgs::Path post_smooth_plan_world = producePathMapToWorld(post_smooth_plan);
    publishPath(plan_publisher_pose_smooth, post_smooth_plan_world);

    // Uniform path Spacing
    std::vector<geometry_msgs::PoseStamped> uniform_plan = uniformSpacing(post_smooth_plan);
    nav_msgs::Path uniform_plan_world = producePathMapToWorld(uniform_plan);
    publishPath(plan_publisher_uniform, uniform_plan_world);

    // TBD: Adaptive path spacing

    // Optimizatoin - use Eigen::Vector2d, instead of geometry_msgs::PoseStamped.
    std::vector<Eigen::Vector2d> uniform_path = convertPoseStampedToVector2d(uniform_plan);
    std::vector<Eigen::Vector2d> opt_path = optimize_min_curvature(uniform_path, Constant::MAX_OPT_ITER);
    std::vector<geometry_msgs::PoseStamped> final_plan = convertVector2dToPoseStamped(opt_path);

    nav_msgs::Path final_plan_world = producePathMapToWorld(final_plan);
    publishPath(plan_publisher_final, final_plan_world);
    
    // Velocity Profile
    Eigen::VectorXd curvature = getCurvatureUsingTurningRadius(final_plan_world);
    auto velocity_result = getVelocityProfile(curvature, Constant::INIT_VELOCITY, Constant::MAX_VEL, Constant::MAX_AX, Constant::MAX_AY);
    Eigen::VectorXd velocity_profile = velocity_result.first;
    Eigen::MatrixXd velocity_constraints = velocity_result.second;

    // Publish velocity profile and constraints for plotting in Python node
    std_msgs::Float64MultiArray profile_msg;
    for (int i = 0; i < velocity_profile.size(); ++i) {
        profile_msg.data.push_back(velocity_profile(i));
    }
    profile_publisher.publish(profile_msg);

    std_msgs::Float64MultiArray constraints_msg;
    for (int i = 0; i < velocity_constraints.rows(); ++i) {
        for (int j = 0; j < velocity_constraints.cols(); ++j) {
            constraints_msg.data.push_back(velocity_constraints(i, j));
        }
    }
    constraints_publisher.publish(constraints_msg);

    plan = final_plan_world.poses;

    return a_star_success;
}

void publishPath(const ros::Publisher& publisher, nav_msgs::Path& path_msg) {
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();    
    publisher.publish(path_msg);
}

void publishVelocityProfile(const ros::Publisher& profile_pub, const Eigen::VectorXd& velocity_profile) {
    std_msgs::Float64MultiArray profile_msg;
    for (int i = 0; i < velocity_profile.size(); ++i) {
        profile_msg.data.push_back(velocity_profile(i));
    }
    profile_pub.publish(profile_msg);
}

void publishVelocityConstraints(const ros::Publisher& constraints_pub, const Eigen::MatrixXd& velocity_constraints) {
    std_msgs::Float64MultiArray constraints_msg;
    for (int i = 0; i < velocity_constraints.rows(); ++i) {
        for (int j = 0; j < velocity_constraints.cols(); ++j) {
            constraints_msg.data.push_back(velocity_constraints(i, j));
        }
    }
    constraints_pub.publish(constraints_msg);
}

nav_msgs::Path producePathMapToWorld(std::vector<geometry_msgs::PoseStamped>& plan) {
    nav_msgs::Path new_plan;
    geometry_msgs::PoseStamped new_pose;
    double world_x, world_y;
    double next_world_x, next_world_y;

    new_pose.header.frame_id = "map";
    new_pose.header.stamp = ros::Time::now();

    if (plan.empty()) {
        ROS_WARN("Fail to produce the path: Empty");
        return new_plan;
    }

    for (size_t i  = 0; i < plan.size() - 1; ++i) {

        costmap->mapToWorld(plan[i].pose.position.x, plan[i].pose.position.y, world_x, world_y);
        new_pose.pose.position.x = world_x;
        new_pose.pose.position.y = world_y;
        new_pose.pose.position.z = 0;
        new_pose.pose.orientation = plan[i].pose.orientation;

        new_plan.poses.push_back(new_pose);
    }

    return new_plan;
}

std::vector<Eigen::Vector2d> convertPoseStampedToVector2d(const std::vector<geometry_msgs::PoseStamped>& plan) {
    std::vector<Eigen::Vector2d> path;
    path.reserve(plan.size());

    for (const auto& poseStamped : plan) {
        path.emplace_back(poseStamped.pose.position.x, poseStamped.pose.position.y);
    }

    return path;
}

std::vector<geometry_msgs::PoseStamped> convertVector2dToPoseStamped(const std::vector<Eigen::Vector2d>& path) {
    std::vector<geometry_msgs::PoseStamped> plan;
    plan.reserve(path.size());

    for (const auto& point : path) {
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.pose.position.x = point.x();
        poseStamped.pose.position.y = point.y();
        poseStamped.pose.position.z = 0.0;

        plan.push_back(poseStamped);
    }

    updateOrientatoin(plan);

    return plan;
}


} // namespace my_global_planner