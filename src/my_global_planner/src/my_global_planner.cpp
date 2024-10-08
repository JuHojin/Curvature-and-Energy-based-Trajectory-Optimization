// my_global_planner.cpp
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include <cmath>
#include <numeric>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "my_global_planner.h"
#include "a_star.h"
#include "path_func.h"
#include "opt_curv.h"
#include "constant.h"
#include "boundaries.h"
#include "my_global_planner/PinPoints.h"
#include "my_global_planner/BSplineSmoothing.h"

// Register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(my_global_planner::MyGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace my_global_planner {

// Global variables
costmap_2d::Costmap2D* costmap = nullptr;
nav_msgs::OccupancyGrid::ConstPtr local_costmap_msg;

// ========================================
// MyGlobalPlanner Class Init.
// ========================================
MyGlobalPlanner::MyGlobalPlanner() : tf_buffer_(), tf_listener_(tf_buffer_, true) {}

MyGlobalPlanner::MyGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : tf_buffer_(), tf_listener_(tf_buffer_, true) {
    initialize(name, costmap_ros);
}

MyGlobalPlanner::~MyGlobalPlanner() {}

void MyGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    costmap = costmap_ros->getCostmap();
    

    replanning_mode = false;
    
    ros::NodeHandle private_nh("~/" + name);
    // Publisher
    plan_publisher_a_star = private_nh.advertise<nav_msgs::Path>("plan/a_star_plan", 1);
    plan_publisher_pose_smooth = private_nh.advertise<nav_msgs::Path>("plan/post_smooth", 1);
    plan_publisher_uniform = private_nh.advertise<nav_msgs::Path>("plan/uniform", 1);
    plan_publisher_bnd_L = private_nh.advertise<nav_msgs::Path>("plan/bnd_L", 1);
    plan_publisher_bnd_R = private_nh.advertise<nav_msgs::Path>("plan/bnd_R", 1);
    plan_publisher_bnd_L_replan = private_nh.advertise<nav_msgs::Path>("plan/bnd_L_replan", 1);
    plan_publisher_bnd_R_replan = private_nh.advertise<nav_msgs::Path>("plan/bnd_R_replan", 1);
    plan_publisher_opt = private_nh.advertise<nav_msgs::Path>("plan/opt", 1);
    plan_publisher_last_opt = private_nh.advertise<nav_msgs::Path>("plan/last_opt", 1);
    plan_publisher_initial = private_nh.advertise<nav_msgs::Path>("plan/initial", 1);
    plan_publisher_replan = private_nh.advertise<nav_msgs::Path>("plan/replan", 1);
    plan_publisher_latest = private_nh.advertise<nav_msgs::Path>("plan/latest", 1);

    profile_publisher = private_nh.advertise<std_msgs::Float64MultiArray>("velocity_profile", 10);
    constraints_publisher = private_nh.advertise<std_msgs::Float64MultiArray>("velocity_constraints", 10);
    pin_points_publisher = private_nh.advertise<my_global_planner::PinPoints>("pin_points", 10);
    replan_status_publisher = private_nh.advertise<std_msgs::Bool>("replan_status", 10);
    planning_calculation_time_publisher = private_nh.advertise<std_msgs::Float64>("planning_calculation_time", 1);

    // Subscriber
    ros::NodeHandle nh;
    local_costmap_sub = nh.subscribe("/move_base/local_costmap/costmap", 10, &MyGlobalPlanner::localCostmapCallback, this);

    // Service
    replan_service_ = nh.advertiseService("replanning_service", &MyGlobalPlanner::handleReplanningRequest, this);

    ROS_INFO("MyGlobalPlanner initialized successfully.");
}

// ========================================
// Callback Functions
// ========================================
bool MyGlobalPlanner::handleReplanningRequest(my_global_planner::ReplanningService::Request &req,
                                              my_global_planner::ReplanningService::Response &res) {

    ROS_INFO("ReplanningService Request received:");
    ROS_INFO("replanning_status: %s", req.replanning_status ? "true" : "false");
    ROS_INFO("current_idx: %d", req.current_idx);
    ROS_INFO("start_idx: %d", req.start_idx);
    ROS_INFO("ahead_idx: %d", req.ahead_idx);
    ROS_INFO("replanning_idx: %d", req.replanning_idx);

    replanning_mode = req.replanning_status;
    received_pin_point_indices_ = {req.current_idx, req.start_idx, req.ahead_idx, req.replanning_idx, req.end_idx};

    if (replanning_mode) {
        ROS_INFO("Replanning Triggered via service");

        if (!start_.header.frame_id.empty() && !goal_.header.frame_id.empty()) {
            bool success = makePlan(start_, goal_, latest_plan_global);
            if (success) {
                ROS_INFO("Replanning successful");
                res.success = true;
                res.replanned_path = replanned_path_world;
            } else {
                ROS_WARN("Replanning failed");
                res.success = false;
            }
        } else {
            ROS_ERROR("Start or Goal not set for replanning");
            res.success = false;
        }
    } else {
        res.success = false;
    }

    replanning_mode = false;
    return true;
}

void MyGlobalPlanner::localCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    local_costmap_msg = msg;
    // ROS_INFO("Local costmap received with size: %d x %d", msg->info.width, msg->info.height);
}

// ========================================
// Main: makePlan
// ========================================
bool MyGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {

    if (local_costmap_msg->data.empty()) {
        ROS_WARN("Local costmap not received yet. Skipping local costmap usage.");
    }

    int planner_type;
    ros::NodeHandle nh("~");
    if (!nh.getParam("/move_base/planner_type", planner_type)) {
        ROS_ERROR("planner_type parameter not set");
        return false;
    }

    if (planner_type < 0 || planner_type > 2) {
        ROS_ERROR("Invalid planner_type parameter: %d. Expected values: 0 (A*), 1 (B-spline), 2 (QP family)", planner_type);
        return false;
    }

    // Minimum curvature planning
    if (!replanning_mode) {
        ROS_INFO("Normal planning mode.");
        start_ = start;
        goal_ = goal;
        plan.clear();

        // Measure planning calculation time: start
        ros::Time start_time = ros::Time::now();

        // Convert start and goal point: (Real) World -> Map
        unsigned int start_x, start_y, goal_x, goal_y;
        bool valid_start = convertWorldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y);
        bool valid_goal = convertWorldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y);
        
        // Check if start and goal is out of bound
        if (!valid_start || !valid_goal) {
            ROS_WARN("Start or goal coordinates are out of bounds. Unable to proceed with A*.");
            return false;
        }

        // Check if start is valid
        unsigned char start_cost = costmap->getCost(start_x, start_y);
        if (start_cost >= Constant::OBSTACLE_INFLATION) {
            ROS_WARN("Start position is inside an obstacle or inflation layer. Skipping replanning.");
            return false;
        }

        // Check if goal is valid
        unsigned char goal_cost = costmap->getCost(goal_x, goal_y);
        if (goal_cost >= Constant::OBSTACLE_INFLATION) {
            ROS_WARN("Goal position is inside an obstacle or inflation layer. Skipping planning.");
            return false;
        }

        // Find initial Path: A* Planning
        ROS_INFO("Find path using A*.");
        basic_planner::AStarPlanner a_star_planner;
        std::vector<geometry_msgs::PoseStamped> a_star_plan;

        bool a_star_success = a_star_planner.makePlan(start_x, start_y, goal_x, goal_y, a_star_plan);
        updateOrientation(a_star_plan);
        nav_msgs::Path a_star_plan_world = producePathMapToWorld(a_star_plan);

        if (a_star_success) {
            ROS_INFO("A* planning is successful.");
            publishPath(plan_publisher_a_star, a_star_plan_world);
        } else {
            ROS_WARN("A* planning failed.");
        }

        // Post smoothing
        std::vector<geometry_msgs::PoseStamped> post_smooth_plan = postSmooth(a_star_plan);
        nav_msgs::Path post_smooth_plan_world = producePathMapToWorld(post_smooth_plan);
        publishPath(plan_publisher_pose_smooth, post_smooth_plan_world);

        // Uniform path Spacing
        std::vector<geometry_msgs::PoseStamped> uniform_plan = uniformSpacing(post_smooth_plan);
        nav_msgs::Path uniform_plan_world = producePathMapToWorld(uniform_plan);
        publishPath(plan_publisher_uniform, uniform_plan_world);
        

        nav_msgs::Path final_plan_world;
        if (planner_type == 0) {
            // A* only
            final_plan_world = uniform_plan_world;
            publishPath(plan_publisher_latest, final_plan_world);
        } else if (planner_type == 1) {
            // B-spline
            ros::NodeHandle nh;
            ros::ServiceClient client = nh.serviceClient<my_global_planner::BSplineSmoothing>("/bspline_smoothing_service");
            my_global_planner::BSplineSmoothing srv;
            srv.request.input_plan = uniform_plan_world;

            if (!client.call(srv)) {
                ROS_ERROR("Failed to call B-spline smoothing service.");
                return false;
            }

            if (srv.response.output_plan.poses.empty()) {
                ROS_ERROR("Received an empty B-spline path from the service.");
                return false;
            }

            final_plan_world = srv.response.output_plan;

            ROS_INFO("Successfully received B-spline path from the service.");
            publishPath(plan_publisher_latest, final_plan_world);
        }
        else if (planner_type == 2) {
            // QP optimization
            // Optimizatoin - use Eigen::Vector2d, instead of geometry_msgs::PoseStamped.
            std::vector<Eigen::Vector2d> uniform_path = convertPoseStampedToVector2d(uniform_plan);
            std::vector<Eigen::Vector2d> opt_path = optimize_min_curvature(uniform_path, Constant::MAX_OPT_ITER, false, tf_buffer_);

            // Pin point selection (based on opt_path)
            auto pin_point_result = pin_point_select(opt_path);
            std::vector<int> opt_pin_point_indices = pin_point_result.first;
            geometry_msgs::PoseArray opt_pin_point_list = pin_point_result.second;

            // Final plan - make opt_path uniform
            std::vector<geometry_msgs::PoseStamped> last_opt_plan = convertVector2dToPoseStamped(opt_path);
            std::vector<geometry_msgs::PoseStamped> final_plan = uniformSpacing(last_opt_plan);
            final_plan_world = producePathMapToWorld(final_plan);
            publishPath(plan_publisher_initial, final_plan_world);
            publishPath(plan_publisher_latest, final_plan_world);
            // for (size_t i = 0; i < final_plan_world.poses.size(); ++i) {
            //     const geometry_msgs::PoseStamped& pose = final_plan_world.poses[i];
            //     std::cout << "Pose[" << i << "]: (" << pose.pose.position.x << ", " << pose.pose.position.y << ")" << std::endl;
            // }

            // Update pin points (based on final_plan)
            auto updated_pin_points = updatePinPoints(opt_pin_point_list, final_plan);
            my_global_planner::PinPoints pin_points_msg;
            pin_points_msg.pose_array = updated_pin_points.first;
            pin_points_msg.index_array.data.clear();
            pin_points_msg.index_array.data.insert(pin_points_msg.index_array.data.end(), updated_pin_points.second.begin(), updated_pin_points.second.end());
            pin_points_publisher.publish(pin_points_msg);

            latest_plan_global = final_plan;
        }

        // Calculate velocity Profile
        Eigen::VectorXd curvature = getCurvatureUsingFiniteDifference(final_plan_world);
        // for (int i = 0; i < curvature.size(); ++i) {
        //     std::cout << "Curvature[" << i << "]: " << curvature[i] << std::endl;
        // }
        auto velocity_result = getVelocityProfile(curvature, Constant::INIT_VELOCITY, Constant::MAX_VEL, Constant::MAX_AX, Constant::MAX_AY);
        Eigen::VectorXd velocity_profile = velocity_result.first;
        Eigen::MatrixXd velocity_constraints = velocity_result.second;
        publishVelocityProfile(profile_publisher, velocity_profile);
        publishVelocityConstraints(constraints_publisher, velocity_constraints);
        // Measure planning calculation time: end
        ros::Time end_time = ros::Time::now();
        double planning_calculation_time = (end_time - start_time).toSec();
        std_msgs::Float64 planning_calculation_time_msg;
        planning_calculation_time_msg.data = planning_calculation_time;
        planning_calculation_time_publisher.publish(planning_calculation_time_msg);

        plan = final_plan_world.poses;
        return a_star_success;
    }
    // Replanning
    else {
        ROS_INFO("Replanning mode.");
        
        if (received_pin_point_indices_.size() < 5) {
            ROS_ERROR("indices list size is wrong");
        }
        int current_idx = received_pin_point_indices_[0];
        int start_idx = received_pin_point_indices_[1];
        int ahead_idx = received_pin_point_indices_[2];
        int pin_point_idx = received_pin_point_indices_[3];
        int end_point_idx = received_pin_point_indices_[4];

        std::vector<geometry_msgs::PoseStamped> plan_initial_global_map;
        std::vector<geometry_msgs::PoseStamped> plan_replan1_global_map;
        std::vector<geometry_msgs::PoseStamped> plan_replan2_global_map;
        std::vector<geometry_msgs::PoseStamped> plan_replan3_global_map;
        std::vector<Eigen::Vector2d> path_replan_global_map;
        std::vector<Eigen::Vector2d> path_replan1_global_map;
        std::vector<Eigen::Vector2d> path_replan2_global_map;
        std::vector<Eigen::Vector2d> path_replan3_global_map;
        std::vector<Eigen::Vector2d> path_replan_local_map;
        std::vector<Eigen::Vector2d> path_opt_replan_local_map;
        std::vector<geometry_msgs::PoseStamped> plan_opt_replan;
        std::vector<geometry_msgs::PoseStamped> plan_final;
        
        ROS_INFO("size: %lu", latest_plan_global.size());
        ROS_INFO("Indices: current_idx = %d, start_idx = %d, ahead_idx = %d, pin_point_idx = %d, end_point_idx = %d", current_idx, start_idx, ahead_idx, pin_point_idx, end_point_idx);

        // 1. latest_plan_global[0] ~ latest_plan_global[start_idx - 1]
        if (start_idx <= latest_plan_global.size()) {
            plan_initial_global_map.insert(plan_initial_global_map.end(), latest_plan_global.begin(), latest_plan_global.begin() + start_idx);
        }

        // 2.1. latest_plan_global[start_idx] ~ latest_plan_global[ahead_idx - 1]
        if (ahead_idx <= latest_plan_global.size()) {
            plan_replan1_global_map.insert(plan_replan1_global_map.end(), latest_plan_global.begin() + start_idx, latest_plan_global.begin() + ahead_idx);
            path_replan1_global_map = convertPoseStampedToVector2d(plan_replan1_global_map);
        }

        // 2.2. Uniformly spaced points between latest_plan_global[ahead_idx] and latest_plan_global[pin_point_idx]
        if (ahead_idx < latest_plan_global.size() && pin_point_idx < latest_plan_global.size()) {
            plan_replan2_global_map.push_back(latest_plan_global[ahead_idx]);
            plan_replan2_global_map.push_back(latest_plan_global[pin_point_idx]);
            path_replan2_global_map = convertPoseStampedToVector2d(plan_replan2_global_map);
        }
        // path_replan2_global_map = uniformSpacing(path_replan2_global_map);

        // 2.3 latest_plan_global[pin_point_idx + 1] ~ latest_plan_global[end_point_idx]
        if (pin_point_idx < end_point_idx && end_point_idx < latest_plan_global.size()) {
            ROS_INFO("%d < %d", pin_point_idx, end_point_idx);
            plan_replan3_global_map.insert(plan_replan3_global_map.end(), latest_plan_global.begin() + pin_point_idx + 1, latest_plan_global.begin() + end_point_idx);
            path_replan3_global_map = convertPoseStampedToVector2d(plan_replan3_global_map);
        }

        // 2.4. combine path_replan1_global_map and path_replan2_global_map to path_replan_global_map
        path_replan_global_map.reserve(path_replan1_global_map.size() + path_replan2_global_map.size() + path_replan3_global_map.size());
        path_replan_global_map.insert(path_replan_global_map.end(), path_replan1_global_map.begin(), path_replan1_global_map.end());
        path_replan_global_map.insert(path_replan_global_map.end(), path_replan2_global_map.begin(), path_replan2_global_map.end());
        if (pin_point_idx < end_point_idx) {
            path_replan_global_map.insert(path_replan_global_map.end(), path_replan3_global_map.begin(), path_replan3_global_map.end());
        }
        path_replan_global_map = uniformSpacing(path_replan_global_map);
        
        // 2.5. Optimizatoin
        float resolution = local_costmap_msg->info.resolution;
        double local_costmap_x = local_costmap_msg->info.origin.position.x;
        double local_costmap_y = local_costmap_msg->info.origin.position.y;

        for (const auto& pose_global_map : path_replan_global_map) {
            // 1. map to world using costmap
            double wx, wy;
            convertMapToWorld(pose_global_map.x(), pose_global_map.y(), wx, wy);
            Eigen::Vector2d point_global_world(wx, wy);
            // ROS_INFO("Global Point in World Frame: (%.2f, %.2f)", wx, wy);

            // 2. Global -> Local (map frame -> odom frame)
            Eigen::Vector2d point_local_world = globalToLocal(point_global_world, *local_costmap_msg, "odom", tf_buffer_);

            // 3. world to map using local_costmap_msg
            Eigen::Vector2d point_local_map;
            point_local_map.x() = (point_local_world.x() - local_costmap_x) / resolution;
            point_local_map.y() = (point_local_world.y() - local_costmap_y) / resolution;

            // ROS_INFO("Transformed local_point: %.2f, %.2f -> Map Coordinates: (%d, %d)", point_local_world.x(), point_local_world.y(), local_map_x, local_map_y);
            path_replan_local_map.push_back(point_local_map);
        }

        path_opt_replan_local_map = optimize_min_curvature(path_replan_local_map, Constant::MAX_OPT_ITER, true, tf_buffer_);

        // ROS_INFO("path_opt_replan_local_map contains %lu points", path_opt_replan_local_map.size());
        // for (size_t i = 0; i < path_opt_replan_local_map.size(); ++i) {
        //     const Eigen::Vector2d& point = path_opt_replan_local_map[i];
        //     ROS_INFO("Point %lu: (x: %.2f, y: %.2f)", i, point.x(), point.y());
        // }

        for (const auto& point_local_map : path_opt_replan_local_map) {
            // 1. map to world using local_costmap_msg
            Eigen::Vector2d local_world_point;
            local_world_point.x() = point_local_map.x() * resolution + local_costmap_x;
            local_world_point.y() = point_local_map.y() * resolution + local_costmap_y;
            // ROS_INFO("Step 1: Local World Point: (%.2f, %.2f)", local_world_point.x(), local_world_point.y());

            // 2. Local -> Global (odom frame -> map frame)
            geometry_msgs::PoseStamped pose_global_world = localToGlobal(local_world_point, *local_costmap_msg, "odom", tf_buffer_);
            // ROS_INFO("Step 2: Transformed Global Pose: (%.2f, %.2f)", pose_global_world.pose.position.x, pose_global_world.pose.position.y);

            // 3. world to map using costmap
            double mx, my;
            if (convertWorldToMap(pose_global_world.pose.position.x, pose_global_world.pose.position.y, mx, my)) {
                // ROS_INFO("Step 3: Map Coordinates (mx, my): (%.2f, %.2f)", mx, my);

                pose_global_world.pose.position.x = mx;
                pose_global_world.pose.position.y = my;
                pose_global_world.pose.position.z = 0.0;
                pose_global_world.pose.orientation.w = 1.0;

                plan_opt_replan.push_back(pose_global_world);

                // ROS_INFO("Global pose added: (%.2f, %.2f)", pose_global_world.pose.position.x, pose_global_world.pose.position.y);
            } else {
                // ROS_WARN("Step 3: Point (%.2f, %.2f) is out of global costmap bounds", pose_global_world.pose.position.x, pose_global_world.pose.position.y);
            }
        }

        // 3. latest_plan_global[pin_point_idx + 1] ~ latest_plan_global[latest_plan_global.back()]
        if (end_point_idx < latest_plan_global.size() - 1) {
            plan_final.insert(plan_final.end(), latest_plan_global.begin() + end_point_idx + 1, latest_plan_global.end());
        } else {
            ROS_WARN("pin_point_idx + 1 is out of bounds. Skipping final latest_plan_global insertion.");
            ROS_INFO("pin_point_idx: %d, latest_plan_global size: %lu", pin_point_idx, latest_plan_global.size());
        }

        // Combine all plans into the latest_plan_global
        latest_plan_global.clear();
        latest_plan_global.insert(latest_plan_global.end(), plan_initial_global_map.begin(), plan_initial_global_map.end());
        latest_plan_global.insert(latest_plan_global.end(), plan_opt_replan.begin(), plan_opt_replan.end());
        latest_plan_global.insert(latest_plan_global.end(), plan_final.begin(), plan_final.end());
        updateOrientation(latest_plan_global);

        replanned_path_world = producePathMapToWorld(latest_plan_global);
        // for (size_t i = 0; i < latest_plan_global.size(); ++i) {
        //     const geometry_msgs::PoseStamped& pose = latest_plan_global[i];
        //     ROS_INFO("Pose %lu: Position (x: %f, y: %f)", i, pose.pose.position.x, pose.pose.position.y);
        // }

        // Calculate velocity Profile
        Eigen::VectorXd curvature = getCurvatureUsingFiniteDifference(replanned_path_world);
        auto velocity_result = getVelocityProfile(curvature, Constant::INIT_VELOCITY, Constant::MAX_VEL, Constant::MAX_AX, Constant::MAX_AY);
        Eigen::VectorXd velocity_profile = velocity_result.first;
        Eigen::MatrixXd velocity_constraints = velocity_result.second;
        publishVelocityProfile(profile_publisher, velocity_profile);
        publishVelocityConstraints(constraints_publisher, velocity_constraints);

        publishPath(plan_publisher_replan, replanned_path_world);
        publishPath(plan_publisher_latest, replanned_path_world);

        return true;
    }
}

// ========================================
// Pin Point Selection
// ========================================
std::pair<std::vector<int>, geometry_msgs::PoseArray> pin_point_select(const std::vector<Eigen::Vector2d>& path) {
    std::vector<int> pin_point_indices;
    std::vector<double> distances(path.size() - 1);
    std::vector<double> average_distance(path.size() - 1);
    int neighbor_range = Constant::NEIGHBOR_RANGE;

    // Calculate distances between consecutive points
    for (size_t i = 1; i < path.size() - 1; ++i) {
        distances[i] = (path[i] - path[i - 1]).norm();
    }

    // Global average method
    // double average_distance = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size() * 0.8;  // Tighter bound

    // Local average method
    for (size_t i = 0; i < path.size() - 1; ++i) {
        double sum_distances = 0.0;
        int valid_neighbors = 0;

        int start_idx = std::max(static_cast<int>(i) - neighbor_range, 0);
        int end_idx = std::min(static_cast<int>(i) + neighbor_range, static_cast<int>(path.size()) - 1);

        for (int j = start_idx; j <= end_idx - 1; ++j) {
            sum_distances += distances[j];
            valid_neighbors++;
        }

        average_distance[i] = sum_distances / valid_neighbors;

    }
    
    // Find indices where distance is below average and group them
    std::vector<int> below_avg_indices;
    for (size_t i = 0; i < distances.size(); ++i) {
        if (distances[i] < average_distance[i]) {
            below_avg_indices.push_back(i);
        }
    }

    std::vector<std::vector<int>> sections;
    std::vector<int> current_section;

    for (size_t i = 0; i < below_avg_indices.size(); ++i) {
        if (i > 0 && below_avg_indices[i] != below_avg_indices[i - 1] + 1) {
            sections.push_back(current_section);
            current_section.clear();
        }
        current_section.push_back(below_avg_indices[i]);
    }
    if (!current_section.empty()) {
        sections.push_back(current_section);
    }

    int method = Constant::PINPOINT_SELECT_METHOD;

    // Method 1: midpoint of each section
    if (method == 1) {
        for (const auto& sec : sections) {
            if (!sec.empty()) {
                pin_point_indices.push_back((sec.front() + sec.back()) / 2);
            }
        }
    }
    
    // Method 2: midpoint of minimum points of each section
    else if (method == 2) {
        for (const auto& sec : sections) {
            if (!sec.empty()) {
                double min_distance = *std::min_element(sec.begin(), sec.end(), [&](int a, int b) {
                    return distances[a] < distances[b];
                });
                int middle_index = sec[std::distance(distances.begin(), std::find(distances.begin(), distances.end(), min_distance))];
                pin_point_indices.push_back(middle_index);
            }
        }
    }

    // Method 3: weighted point
    else if (method == 3) {
        for (const auto& sec : sections) {
            if (!sec.empty()) {
                std::vector<double> section_distances(sec.size());
                for (size_t j = 0; j < sec.size(); ++j) {
                    section_distances[j] = distances[sec[j]];
                }
                std::vector<double> weights(sec.size());
                std::transform(section_distances.begin(), section_distances.end(), weights.begin(), [](double d) {
                    return 1.0 / (std::pow(d, 3) + 1e-6);
                });
                double weighted_index = std::inner_product(sec.begin(), sec.end(), weights.begin(), 0.0) / std::accumulate(weights.begin(), weights.end(), 0.0);
                pin_point_indices.push_back(static_cast<int>(std::round(weighted_index)));
            }
        }
    }

    // Ensure the last point is included
    int last_idx = distances.size() - 1;
    if (pin_point_indices.back() != last_idx) {
        pin_point_indices.push_back(last_idx);
    }

    // Convert selected pin points to geometry_msgs::PoseArray
    geometry_msgs::PoseArray pose_array;
    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = "map";

    for (int i : pin_point_indices) {
        geometry_msgs::Pose pose;
        pose.position.x = path[i][0];
        pose.position.y = path[i][1];
        pose.position.z = 0.0;
        pose.orientation.w = 1.0;
        pose_array.poses.push_back(pose);
    }

    return {pin_point_indices, pose_array};
}

std::pair<geometry_msgs::PoseArray, std::vector<int>> updatePinPoints(
    const geometry_msgs::PoseArray& opt_pin_point_list,
    const std::vector<geometry_msgs::PoseStamped>& final_plan) {

    geometry_msgs::PoseArray updated_pin_point_list;
    std::vector<int> updated_pin_point_indices;

    for (const auto& pin_point : opt_pin_point_list.poses) {
        double min_distance = std::numeric_limits<double>::max();
        int closest_index = -1;

        for (size_t i = 0; i < final_plan.size(); ++i) {
            double dx = final_plan[i].pose.position.x - pin_point.position.x;
            double dy = final_plan[i].pose.position.y - pin_point.position.y;
            double distance = std::sqrt(dx * dx + dy * dy);

            if (distance < min_distance) {
                min_distance = distance;
                closest_index = i;
            }
        }

        if (closest_index != -1) {
            updated_pin_point_list.poses.push_back(final_plan[closest_index].pose);
            updated_pin_point_indices.push_back(closest_index);
        }
    }

    return {updated_pin_point_list, updated_pin_point_indices};
}

// ========================================
// Publish Functions
// ========================================
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

// ========================================
// Convert Functions
// ========================================
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

    double origin_x = costmap->getOriginX();
    double origin_y = costmap->getOriginY();
    double resolution = costmap->getResolution();

    for (size_t i  = 0; i < plan.size() - 1; ++i) {

        new_pose.pose.position.x = plan[i].pose.position.x * resolution + origin_x;
        new_pose.pose.position.y = plan[i].pose.position.y * resolution + origin_y;
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

    updateOrientation(plan);

    return plan;
}

bool convertMapToWorld(const unsigned int& mx, const unsigned int& my, double& wx, double& wy) {
    double origin_x = costmap->getOriginX();
    double origin_y = costmap->getOriginY();
    double resolution = costmap->getResolution();
    
    unsigned int map_width = costmap->getSizeInCellsX();
    unsigned int map_height = costmap->getSizeInCellsY();

    if (mx >= map_width || my >= map_height) {
        return false;
    }

    wx = mx * resolution + origin_x;
    wy = my * resolution + origin_y;

    return true;
}

bool convertMapToWorld(const double& mx, const double& my, double& wx, double& wy) {
    double origin_x = costmap->getOriginX();
    double origin_y = costmap->getOriginY();
    double resolution = costmap->getResolution();
    
    unsigned int map_width = costmap->getSizeInCellsX();
    unsigned int map_height = costmap->getSizeInCellsY();

    if (mx >= map_width || my >= map_height) {
        return false;
    }

    wx = mx * resolution + origin_x;
    wy = my * resolution + origin_y;

    return true;
}

bool convertWorldToMap(const double& wx, const double& wy, unsigned int &mx, unsigned int &my) {
    double origin_x = costmap->getOriginX();
    double origin_y = costmap->getOriginY();
    double resolution = costmap->getResolution();
    
    unsigned int map_width = costmap->getSizeInCellsX();
    unsigned int map_height = costmap->getSizeInCellsY();

    if (wx < origin_x || wy < origin_y) {
        return false;
    }

    mx = static_cast<int>((wx - origin_x) / resolution);
    my = static_cast<int>((wy - origin_y) / resolution);

    if (mx < map_width && my < map_height) {
        return true;
    } else {
        return false;
    }
}

bool convertWorldToMap(const double& wx, const double& wy, double &mx, double &my) {
    double origin_x = costmap->getOriginX();
    double origin_y = costmap->getOriginY();
    double resolution = costmap->getResolution();
    
    unsigned int map_width = costmap->getSizeInCellsX();
    unsigned int map_height = costmap->getSizeInCellsY();

    if (wx < origin_x || wy < origin_y) {
        return false;
    }

    mx = (wx - origin_x) / resolution;
    my = (wy - origin_y) / resolution;

    if (mx < map_width && my < map_height) {
        return true;
    } else {
        return false;
    }
}

geometry_msgs::PoseStamped localToGlobal(const Eigen::Vector2d& local_point, const nav_msgs::OccupancyGrid& local_costmap_msg, const std::string& target_frame, tf2_ros::Buffer& tf_buffer_) {
    geometry_msgs::PoseStamped local_pose;
    geometry_msgs::PoseStamped global_pose;

    double origin_x = local_costmap_msg.info.origin.position.x;
    double origin_y = local_costmap_msg.info.origin.position.y;
    double resolution = local_costmap_msg.info.resolution;

    local_pose.pose.position.x = local_point.x();
    local_pose.pose.position.y = local_point.y();
    local_pose.pose.position.z = 0.0;
    local_pose.pose.orientation.w = 1.0;
    local_pose.header.frame_id = target_frame;
    local_pose.header.stamp = ros::Time::now();

    try {
        tf_buffer_.transform(local_pose, global_pose, "map", ros::Duration(1.0));

        // ROS_INFO("Transformed local pose to global pose: (%.2f, %.2f)", global_pose.pose.position.x, global_pose.pose.position.y);
    } catch (tf2::TransformException& ex) {
        ROS_WARN("Transform failed: %s", ex.what());
    }

    return global_pose;
}

Eigen::Vector2d globalToLocal(const Eigen::Vector2d& global_point, const nav_msgs::OccupancyGrid& local_costmap_msg, const std::string& target_frame, tf2_ros::Buffer& tf_buffer_) {
    geometry_msgs::PoseStamped global_pose;
    geometry_msgs::PoseStamped local_pose;
    Eigen::Vector2d local_point;

    global_pose.pose.position.x = global_point.x();
    global_pose.pose.position.y = global_point.y();
    global_pose.pose.position.z = 0.0;
    global_pose.header.frame_id = "map";
    global_pose.header.stamp = ros::Time::now();
    try {
        tf_buffer_.transform(global_pose, local_pose, target_frame, ros::Duration(1.0));

        local_point.x() = local_pose.pose.position.x;
        local_point.y() = local_pose.pose.position.y;
    } catch (tf2::TransformException& ex) {
        ROS_WARN("Transform failed: %s", ex.what());
    }

    return local_point;
}


} // namespace my_global_planner