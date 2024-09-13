// replanning_trigger.cpp
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include "my_global_planner/PinPoints.h"
#include "my_global_planner/ReplanningService.h"
#include "constant.h"

class ReplanningTriggerNode {
public:
    ReplanningTriggerNode() {
        pin_points_sub_ = nh_.subscribe("/move_base/MyGlobalPlanner/pin_points", 1, &ReplanningTriggerNode::pinPointsCallback, this);
        global_costmap_sub_ = nh_.subscribe("/move_base/global_costmap/costmap", 1, &ReplanningTriggerNode::globalCostmapCallback, this);
        local_costmap_sub_ = nh_.subscribe("/move_base/local_costmap/costmap", 1, &ReplanningTriggerNode::localCostmapCallback, this);
        amcl_pose_sub_ = nh_.subscribe("/amcl_pose", 1, &ReplanningTriggerNode::amclPoseCallback, this);
        velocity_sub_ = nh_.subscribe("/car_1/base/odom", 1, &ReplanningTriggerNode::velocityCallback, this);
        initial_plan_world_sub_ = nh_.subscribe("/move_base/MyGlobalPlanner/plan/initial", 1, &ReplanningTriggerNode::initialPlanCallback, this);

        pin_points_pub_ = nh_.advertise<visualization_msgs::Marker>("/replanning_trigger/pin_points_marker", 1);
        valid_pin_points_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/replanning_trigger/valid_pin_points", 1);
        fixed_start_points_pub_ = nh_.advertise<visualization_msgs::Marker>("/replannig_trigger/fixed_start_points_marker", 1);
        fixed_ahead_points_pub_ = nh_.advertise<visualization_msgs::Marker>("/replannig_trigger/fixed_ahead_points_marker", 1);
        fixed_pin_points_pub_ = nh_.advertise<visualization_msgs::Marker>("/replannig_trigger/fixed_pin_points_marker", 1);
        line_pub_ = nh_.advertise<visualization_msgs::Marker>("/replanning_trigger/line_marker", 1);
        start_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/replanning_trigger/start_point", 1);
        ahead_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/replanning_trigger/ahead_point", 1);
        trigger_pub_ = nh_.advertise<std_msgs::Bool>("/move_base/MyGlobalPlanner/replanning_trigger", 1);
        replanning_indices_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("/replanning_trigger/replanning_index", 1);

        global_costmap_ = nullptr;
        local_costmap_ = nullptr;
    }

    void run() {
        int current_idx;
        int start_idx;
        int ahead_idx;
        int replanning_idx;

        ros::Rate rate(1.0);
        while (ros::ok()) {
            ROS_INFO("\n\n");
            ros::spinOnce();
            publishAllPinPoints();
            current_idx = getCurrentIndex();
            auto result = getStartAndAheadIndex(current_idx);
            start_idx = result.first;
            ahead_idx = result.second;

            ROS_INFO("replanning bound idx: %d", replanning_bound_idx_global);

            replanning_bound_idx_global = triggerReplanning(current_idx, start_idx, ahead_idx, replanning_bound_idx_global);
            ROS_INFO("replanning finsih");
            rate.sleep();
            }
        }

private:
    ros::NodeHandle nh_;
    ros::Publisher pin_points_pub_;
    ros::Publisher valid_pin_points_pub_;
    ros::Publisher fixed_start_points_pub_;
    ros::Publisher fixed_ahead_points_pub_;
    ros::Publisher fixed_pin_points_pub_;
    ros::Publisher line_pub_;
    ros::Publisher start_point_pub_;
    ros::Publisher ahead_point_pub_;
    ros::Publisher trigger_pub_;
    ros::Publisher replanning_indices_pub_;
    
    ros::Subscriber pin_points_sub_;
    ros::Subscriber global_costmap_sub_;
    ros::Subscriber local_costmap_sub_;
    ros::Subscriber amcl_pose_sub_;
    ros::Subscriber velocity_sub_;
    ros::Subscriber initial_plan_world_sub_;

    int replanning_bound_idx_global = -1;
    double linear_velocity_global = 0.0;
    bool flag_global = true;

    geometry_msgs::PoseArray pin_points_map_;
    geometry_msgs::PoseArray pin_points_world_;
    std::vector<int> pin_point_indices_reversed_;
    std::vector<int> pin_point_indices_;
    nav_msgs::OccupancyGrid::ConstPtr global_costmap_;
    nav_msgs::OccupancyGrid::ConstPtr local_costmap_;
    geometry_msgs::PointStamped current_pose_;
    nav_msgs::Path current_plan_world_;
    tf::TransformListener tf_listener_;

// ========================================
// Callback Functions
// ========================================
    void velocityCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        double linear_velocity_x = msg->twist.twist.linear.x;
        double linear_velocity_y = msg->twist.twist.linear.y;
        linear_velocity_global = std::hypot(linear_velocity_x, linear_velocity_y);
        // ROS_INFO("Current Linear Velocity: %.2f", linear_velocity_global);
    }

    void pinPointsCallback(const my_global_planner::PinPoints::ConstPtr& msg) {
        if (!global_costmap_) {
            ROS_WARN("Global costmap not received yet.");
            return;
        }

        ROS_INFO("Received PinPoints message with %lu poses and %lu indices",
                msg->pose_array.poses.size(), msg->index_array.data.size());

        pin_points_map_.poses.clear();
        pin_points_world_.poses.clear();

        for (const auto& pose : msg->pose_array.poses) {
            geometry_msgs::Pose transformed_pose = pose;
            double wx, wy;

            mapToWorld(global_costmap_, pose.position.x, pose.position.y, wx, wy);

            transformed_pose.position.x = wx;
            transformed_pose.position.y = wy;

            pin_points_map_.poses.push_back(pose);
            pin_points_world_.poses.push_back(transformed_pose);
        }

        pin_point_indices_reversed_ = msg->index_array.data;
        pin_point_indices_ = msg->index_array.data;
        std::reverse(pin_point_indices_reversed_.begin(), pin_point_indices_reversed_.end());

        ROS_INFO("Processed pin points and indices.");
    }


    void globalCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        global_costmap_ = msg;
    }

    void localCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        local_costmap_ = msg;
    }

    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        current_pose_.point.x = msg->pose.pose.position.x;
        current_pose_.point.y = msg->pose.pose.position.y;
        current_pose_.point.z = msg->pose.pose.position.z;
        current_pose_.header = msg->header;
        // ROS_INFO("Received AMCL pose.");
    }

    void initialPlanCallback(const nav_msgs::Path::ConstPtr& msg) {
        current_plan_world_ = *msg;
        replanning_bound_idx_global = 1;
        ROS_INFO("Recevied initial optimized plan");
    }

// ========================================
// Update Indices
// ========================================
    int getCurrentIndex() {
        int current_idx = -1;
        if (current_plan_world_.poses.empty()) {
            ROS_WARN("Final plan is empty, cannot find the nearest point.");
            return current_idx;
        }

        int nearest_index = -1;
        double min_distance = std::numeric_limits<double>::max();

        double current_x = current_pose_.point.x;
        double current_y = current_pose_.point.y;

        for (int i = 0; i < current_plan_world_.poses.size(); ++i) {
            double plan_x = current_plan_world_.poses[i].pose.position.x;
            double plan_y = current_plan_world_.poses[i].pose.position.y;

            double distance = std::sqrt(std::pow(plan_x - current_x, 2) + std::pow(plan_y - current_y, 2));

            if (distance < min_distance) {
                min_distance = distance;
                nearest_index = i;
            }
        }

        current_idx = nearest_index;
        return current_idx;
    }

    std::pair<int, int> getStartAndAheadIndex(int current_idx) {
        if (current_idx == -1) {
            return {-1, -1};
        }
        double time = 4.0;
        int min_gap = 3;
        int start_idx = current_idx + std::max(static_cast<int>(linear_velocity_global * time), min_gap);
        int ahead_idx = current_idx + 2 * std::max(static_cast<int>(linear_velocity_global * time), min_gap);

        start_idx = std::min(start_idx, static_cast<int>(current_plan_world_.poses.size() - 1));
        ahead_idx = std::min(ahead_idx, static_cast<int>(current_plan_world_.poses.size() - 1));

        publishStartAndAheadPoints(start_idx, ahead_idx);

        return {start_idx, ahead_idx};
    }

    int getReplanningIndex(int current_idx, int replanning_bound_idx) {
        if (current_idx == -1) {
            ROS_WARN("Unable to update replanning index because the nearest point index could not be found.");
            return -1;
        }
        
        // Debug in simulation
        // std::stringstream ss;
        // ss << "Pin point indices: ";
        // for (size_t i = 0; i < pin_point_indices_reversed_.size(); ++i) {
        //     ss << pin_point_indices_reversed_[i];
        //     if (i < pin_point_indices_reversed_.size() - 1) {
        //         ss << ", ";
        //     }
        // }
        // ROS_INFO("%s", ss.str().c_str());
        
        // Update replanning bound index
        while (!(pin_point_indices_reversed_.empty()) && current_idx >= pin_point_indices_reversed_.back()) {
            ROS_INFO("current idx: %d, pin idx: %d / replanning bound idx ++ at updateReplanningIdx", current_idx, pin_point_indices_reversed_.back());
            pin_point_indices_reversed_.pop_back();
            ++replanning_bound_idx;
            if (replanning_bound_idx >= pin_points_map_.poses.size()) {
                replanning_bound_idx = pin_points_map_.poses.size();
            }
        }

        visualization_msgs::MarkerArray marker_array;
        
        // Delete markers for points before the replanning bound index
        for (size_t i = 0; i < replanning_bound_idx; ++i) {
            visualization_msgs::Marker delete_marker = createMarker(
                "map", "valid_pin_points", i, geometry_msgs::Point(), geometry_msgs::Quaternion(), 
                "SPHERE", {0.2, 0.2, 0.2}, {1.0, 0.0, 0.0, 1.0}, "DELETE"
            );
            marker_array.markers.push_back(delete_marker);
            // ROS_INFO("Deleted marker for point %zu", i);
        }

        // Add markers for points after the replanning bound index
        for (size_t i = replanning_bound_idx; i < pin_points_world_.poses.size(); ++i) {
            visualization_msgs::Marker add_marker = createMarker(
                "map", "valid_pin_points", i, pin_points_world_.poses[i].position, 
                geometry_msgs::Quaternion(), "SPHERE", {0.2, 0.2, 0.2}, {0.0, 1.0, 0.0, 1.0}, "ADD"
            );
            marker_array.markers.push_back(add_marker);
            // ROS_INFO("Added marker for point %zu: (%.2f, %.2f, %.2f)", i, 
            //     add_marker.pose.position.x, add_marker.pose.position.y, add_marker.pose.position.z);
        }

        // Publish the entire MarkerArray (add and delete markers)
        valid_pin_points_pub_.publish(marker_array);
        // ROS_INFO("Published %zu markers (including deleted ones)", marker_array.markers.size());

        return replanning_bound_idx;
    }
    
    void updateReplanningIndex() {
        pin_point_indices_.clear();

        for (size_t i = 0; i < pin_points_world_.poses.size(); ++i) {
            const auto& pin_point = pin_points_world_.poses[i].position;

            int closest_idx = -1;
            double min_distance = std::numeric_limits<double>::max();

            for (size_t j = 0; j < current_plan_world_.poses.size(); ++j) {
                const auto& plan_point = current_plan_world_.poses[j].pose.position;
                
                double dx = plan_point.x - pin_point.x;
                double dy = plan_point.y - pin_point.y;
                double distance = std::sqrt(dx * dx + dy * dy);

                if (distance < min_distance) {
                    min_distance = distance;
                    closest_idx = j;
                }
            }

            if (closest_idx != -1) {
                pin_point_indices_.push_back(closest_idx);
            }
        }

        for (size_t i = 0; i < pin_point_indices_reversed_.size(); ++i) {
            pin_point_indices_reversed_[pin_point_indices_reversed_.size() - i - 1] = pin_point_indices_[i];
        }
        
        std::reverse(pin_point_indices_reversed_.begin(), pin_point_indices_reversed_.end());

        ROS_INFO("Updated pin point indices and reversed indices based on the current plan.");
    }


    int findClosestPointIndex(const nav_msgs::Path& plan, const geometry_msgs::Pose& target_pose) {
        int closest_idx = -1;
        double min_distance = std::numeric_limits<double>::max();

        for (int i = 0; i < plan.poses.size(); ++i) {
            const geometry_msgs::Pose& plan_pose = plan.poses[i].pose;

            double distance = std::sqrt(std::pow(plan_pose.position.x - target_pose.position.x, 2) +
                                        std::pow(plan_pose.position.y - target_pose.position.y, 2));

            if (distance < min_distance) {
                min_distance = distance;
                closest_idx = i;
            }
        }

        return closest_idx;
    }

// ========================================
// Replanning
// ========================================
    int triggerReplanning(int current_idx, int start_idx, int ahead_idx, int replanning_bound_idx) {
        if (current_idx == -1 || start_idx == -1 || ahead_idx == -1) {
            ROS_WARN("current %d, start %d, ahead %d indices are not ready", current_idx, start_idx, ahead_idx);
            return replanning_bound_idx;
        }

        replanning_bound_idx = getReplanningIndex(current_idx, replanning_bound_idx);

        auto result = checkForReplanning(start_idx, ahead_idx, replanning_bound_idx);
        bool replan_possible = result.first;
        replanning_bound_idx = result.second;

        for (int i = 0; i < pin_point_indices_.size(); ++i) {
            ROS_INFO("idx: %d", pin_point_indices_[i]);
        }

        int replanning_idx = pin_point_indices_[replanning_bound_idx];
        ROS_INFO("Replan possible: %s", replan_possible ? "true" : "false");
        ROS_INFO("Replanning bound idx %d", replanning_bound_idx);
        if (replan_possible) {
            ros::NodeHandle nh;
            ros::ServiceClient client = nh.serviceClient<my_global_planner::ReplanningService>("replanning_service");
            my_global_planner::ReplanningService srv;

            srv.request.replanning_status = true;
            srv.request.current_idx = current_idx;
            srv.request.start_idx = start_idx;
            srv.request.ahead_idx = ahead_idx;
            srv.request.replanning_idx = replanning_idx;
            srv.request.end_idx = std::min(replanning_idx + (ahead_idx - start_idx), static_cast<int>(current_plan_world_.poses.size() - 1));

            if (client.call(srv)) {
                if (srv.response.success) {
                    ROS_INFO("Replanning request processed successfully");
                    current_plan_world_ = srv.response.replanned_path;
                    ++replanning_bound_idx;
                    updateReplanningIndex();

                } else {
                    ROS_WARN("Replanning request was not successful");
                }
            } else {
                ROS_ERROR("Failed to call service replanning_service. Is the service available?");
            }
        } else {
            ROS_INFO("No need to replan");
        }

        return replanning_bound_idx;
    }


    std::pair<bool, int> checkForReplanning(int start_idx, int ahead_idx, int replanning_bound_idx) {
        if (!global_costmap_ || !local_costmap_) {
            ROS_WARN("Costmaps not received yet.");
            return {false, replanning_bound_idx};
        }

        if (pin_points_world_.poses.empty()) {
            ROS_WARN("Pin points not received yet.");
            return {false, replanning_bound_idx};
        }

        // Obtain ahead point 
        geometry_msgs::PointStamped global_ahead_point_world, local_ahead_point_world;
        geometry_msgs::Point global_ahead_point_map, local_ahead_point_map;
        global_ahead_point_world.header.frame_id = "map";
        global_ahead_point_world.header.stamp = ros::Time(0);
        global_ahead_point_world.point = current_plan_world_.poses[ahead_idx].pose.position;

        // global_ahead_point_world -> global_ahead_point_map
        worldToMap(global_costmap_, global_ahead_point_world.point.x, global_ahead_point_world.point.y, global_ahead_point_map.x, global_ahead_point_map.y);

        // global_ahead_point_world -> local_ahead_point_world
        try {
            // Transform global_ahead_position from "map" frame to "odom" frame (or any other local frame)
            tf_listener_.transformPoint("odom", global_ahead_point_world, local_ahead_point_world);
        } catch (tf2::TransformException& ex) {
            ROS_ERROR("Failed to transform ahead point from 'map' to 'odom': %s", ex.what());
            return {false, replanning_bound_idx};
        }

        // local_ahead_point_world -> local_ahead_point_map
        worldToMap(local_costmap_, local_ahead_point_world.point.x, local_ahead_point_world.point.y, local_ahead_point_map.x, local_ahead_point_map.y);

        for (int i = pin_points_world_.poses.size() - 1; i >= replanning_bound_idx; --i) {
            geometry_msgs::PointStamped global_pin_point_world, local_pin_point_world;
            geometry_msgs::Point global_pin_point_map, local_pin_point_map;

            global_pin_point_world.header.frame_id = "map";
            global_pin_point_world.header.stamp = ros::Time(0);
            global_pin_point_world.point.x = pin_points_world_.poses[i].position.x;
            global_pin_point_world.point.y = pin_points_world_.poses[i].position.y;
            global_pin_point_world.point.z = 0;
            
            // global_pin_point_world -> local_pin_point_world
            try {
                tf_listener_.transformPoint("odom", global_pin_point_world, local_pin_point_world);
            } catch (tf2::TransformException& ex) {
                ROS_ERROR("Transform error: %s", ex.what());
                return {false, replanning_bound_idx};
            }

            // local_pin_point_world -> local_pin_point_map
            worldToMap(local_costmap_, local_pin_point_world.point.x, local_pin_point_world.point.y, local_pin_point_map.x, local_pin_point_map.y);

            if (local_pin_point_map.x < 0 || local_pin_point_map.x >= local_costmap_->info.width || local_pin_point_map.y < 0 || local_pin_point_map.y >= local_costmap_->info.height) {
                ROS_INFO("Pin point[%d] is (%f, %f) out of local costmap bounds (%d, %d), skipping this point.", i, local_pin_point_map.x, local_pin_point_map.y, local_costmap_->info.width, local_costmap_->info.height);
                continue;
            }

            int index = static_cast<int>(local_pin_point_map.y) * local_costmap_->info.width + static_cast<int>(local_pin_point_map.x);

            if (local_costmap_->data[index] != 0) {
                // ROS_INFO("Pin point[%d] is in an obstacle, inflation layer, or unknown region, skipping this point.", i);
                continue;
            }

            if (isPathClear(local_ahead_point_map, local_pin_point_map, local_costmap_)) {
                ROS_INFO("\nPoint (%d, %d)is CLEAR IN LOCAL COSTMAP\n", static_cast<int>(local_pin_point_map.x), static_cast<int>(local_pin_point_map.y));
                if (!isPathClear(global_ahead_point_map, global_pin_point_map, global_costmap_)) {
                    flag_global = true;
                    // visualization
                    publishLineMarker(current_plan_world_.poses[ahead_idx].pose.position, pin_points_world_.poses[i].position);
                    publishMarker(current_plan_world_.poses[start_idx].pose.position, "fixed_start_points", start_idx, 0.2, {32.0/255.0, 74.0/255.0, 135.0/255.0, 1.0}, fixed_start_points_pub_);
                    publishMarker(current_plan_world_.poses[ahead_idx].pose.position, "fixed_ahead_points", ahead_idx, 0.2, {252.0/255.0, 233.0/255.0, 79.0/255.0, 1.0}, fixed_ahead_points_pub_);
                    publishMarker(pin_points_world_.poses[i].position, "fixed_pin_points", i, 0.2, {1.0, 0.0, 0.0, 1.0}, fixed_pin_points_pub_);

                    replanning_bound_idx = i;
                    ROS_INFO("replanning bound idx: %d at PATHCLEAR", replanning_bound_idx);
                    if (replanning_bound_idx >= pin_points_world_.poses.size()) {
                        replanning_bound_idx = pin_points_world_.poses.size();
                    }

                    return {true, replanning_bound_idx};
                }
            }

        }

        ROS_INFO("No need for replanning.");
        return {false, replanning_bound_idx};
    }

    bool isPathClear(const geometry_msgs::Point& start, const geometry_msgs::Point& end, const nav_msgs::OccupancyGrid::ConstPtr& costmap) {
        int start_x = static_cast<int>(start.x);
        int start_y = static_cast<int>(start.y);
        int end_x = static_cast<int>(end.x);
        int end_y = static_cast<int>(end.y);


        int dx = abs(end_x - start_x);
        int dy = abs(end_y - start_y);
        int sx = start_x < end_x ? 1 : -1;
        int sy = start_y < end_y ? 1 : -1;
        int err = dx - dy;

        while (start_x != end_x || start_y != end_y) {
            int index = start_y * costmap->info.width + start_x;
            if (index >= 0 && index < static_cast<int>(costmap->data.size())) {
                unsigned char cost = costmap->data[index];
                if (cost >= Constant::OBSTACLE_INFLATION || cost == costmap_2d::NO_INFORMATION) {
                    return false;  // Obstacle exists
                }
            }

            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                start_x += sx;
            }
            if (e2 < dx) {
                err += dx;
                start_y += sy;
            }
        }

        return true;  // No Obstacle
    }

// ========================================
// Convert
// ========================================
    void worldToMap(const nav_msgs::OccupancyGrid::ConstPtr& costmap, double wx, double wy, double& mx, double& my) {
        double origin_x = costmap->info.origin.position.x;
        double origin_y = costmap->info.origin.position.y;
        double resolution = costmap->info.resolution;

        mx = static_cast<int>((wx - origin_x) / resolution);
        my = static_cast<int>((wy - origin_y) / resolution);
    }

    void mapToWorld(const nav_msgs::OccupancyGrid::ConstPtr& costmap, double mx, double my, double& wx, double& wy) {
        double origin_x = costmap->info.origin.position.x;
        double origin_y = costmap->info.origin.position.y;
        double resolution = costmap->info.resolution;

        wx = mx * resolution + origin_x;
        wy = my * resolution + origin_y;
    }

    void publishLineMarker(const geometry_msgs::Point& start, const geometry_msgs::Point& end) {
        geometry_msgs::Quaternion identity_quat;
        identity_quat.w = 1.0;

        geometry_msgs::Point zero_point;
        zero_point.x = 0.0;
        zero_point.y = 0.0;
        zero_point.z = 0.0;

        visualization_msgs::Marker line_marker = createMarker(
            "map", "replanning_trigger", 0, 
            zero_point, identity_quat, "LINE_STRIP", {0.05, 0.05, 0.05}, {1.0, 0.0, 0.0, 1.0}, "ADD"
        );

        line_marker.points.push_back(start);
        line_marker.points.push_back(end);

        line_pub_.publish(line_marker);
    }

// ========================================
// Publish Functions
// ========================================
    void publishStartAndAheadPoints(int start_idx, int ahead_idx) {
        if (start_idx >= 0 && start_idx < current_plan_world_.poses.size()) {
            geometry_msgs::PointStamped start_point;
            start_point.header.frame_id = "map";
            start_point.header.stamp = ros::Time::now();
            start_point.point = current_plan_world_.poses[start_idx].pose.position;

            start_point_pub_.publish(start_point);
        } else {
            ROS_WARN("Start index out of bounds, cannot publish start point.");
        }

        if (ahead_idx >= 0 && ahead_idx < current_plan_world_.poses.size()) {
            geometry_msgs::PointStamped ahead_point;
            ahead_point.header.frame_id = "map";
            ahead_point.header.stamp = ros::Time::now();
            ahead_point.point = current_plan_world_.poses[ahead_idx].pose.position;

            ahead_point_pub_.publish(ahead_point);
        } else {
            ROS_WARN("Ahead index out of bounds, cannot publish ahead point.");
        }
    }
    
    void publishAllPinPoints() {
        for (size_t i = 0; i < pin_points_world_.poses.size(); ++i) {
            geometry_msgs::Point position = pin_points_world_.poses[i].position;
            geometry_msgs::Quaternion orientation;
            orientation.w = 1.0;

            // Green point marker
            visualization_msgs::Marker points_marker = createMarker(
                "map", "pin_points", i, 
                position, orientation, "SPHERE", {0.15, 0.15, 0.15}, {0.0, 0.0, 0.0, 1.0}
            );

            pin_points_pub_.publish(points_marker);
        }
    }

    visualization_msgs::Marker createMarker(
        const std::string& frame_id, const std::string& ns, int id, 
        const geometry_msgs::Point& position, const geometry_msgs::Quaternion& orientation,
        const std::string& marker_type, const std::array<double, 3>& scale, 
        const std::array<double, 4>& color, const std::string& action = "ADD") {

        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = id;
        
        if (marker_type == "SPHERE") {
            marker.type = visualization_msgs::Marker::SPHERE;
        } else if (marker_type == "LINE_STRIP") {
            marker.type = visualization_msgs::Marker::LINE_STRIP;
        }

        if (action == "ADD") {
            marker.action = visualization_msgs::Marker::ADD;
        } else if (action == "DELETE") {
            marker.action = visualization_msgs::Marker::DELETE;
        }

        if (marker_type != "LINE_STRIP") {
            marker.pose.position = position;
            marker.pose.orientation = orientation;
        } else {
            marker.pose.position.x = 0.0;
            marker.pose.position.y = 0.0;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
        }

        marker.scale.x = scale[0];
        marker.scale.y = scale[1];
        marker.scale.z = scale[2];

        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        marker.color.a = color[3];

        return marker;
    }

    void publishMarker(const geometry_msgs::Point& position, const std::string& ns, int id, double scale, const std::array<double, 4>& color, ros::Publisher& pub) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = position;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;

        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        marker.color.a = color[3];

        pub.publish(marker);
    }


};

// ========================================
// Main
// ========================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "replanning_trigger_node");
    ReplanningTriggerNode node;
    node.run();
    return 0;
}
