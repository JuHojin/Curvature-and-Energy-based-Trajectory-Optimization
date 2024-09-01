#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "constant.h"

class ReplanningTriggerNode {
public:
    ReplanningTriggerNode() {
        trigger_pub_ = nh_.advertise<std_msgs::Bool>("/move_base/MyGlobalPlanner/replanning_trigger", 1);
        pin_point_sub_ = nh_.subscribe("/move_base/MyGlobalPlanner/pin_points", 1, &ReplanningTriggerNode::pinPointCallback, this);
        global_costmap_sub_ = nh_.subscribe("/move_base/global_costmap/costmap", 1, &ReplanningTriggerNode::globalCostmapCallback, this);
        local_costmap_sub_ = nh_.subscribe("/move_base/local_costmap/costmap", 1, &ReplanningTriggerNode::localCostmapCallback, this);
        amcl_pose_sub_ = nh_.subscribe("/amcl_pose", 1, &ReplanningTriggerNode::amclPoseCallback, this);
        line_pub_ = nh_.advertise<visualization_msgs::Marker>("/replanning_trigger/line_marker", 1);
        pin_points_pub_ = nh_.advertise<visualization_msgs::Marker>("/replanning_trigger/pin_points_marker", 1);

        global_costmap_ = nullptr;
        local_costmap_ = nullptr;
    }

    void run() {
        ros::Rate rate(1.0);
        while (ros::ok()) {
            publishAllPinPoints();
            checkForReplanning();
            std_msgs::Bool msg;
            msg.data = trigger_replanning_;
            trigger_pub_.publish(msg);
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher trigger_pub_;
    ros::Publisher line_pub_;
    ros::Publisher pin_points_pub_;
    ros::Subscriber pin_point_sub_;
    ros::Subscriber global_costmap_sub_;
    ros::Subscriber local_costmap_sub_;
    ros::Subscriber amcl_pose_sub_;

    bool trigger_replanning_ = false;
    geometry_msgs::PoseArray pin_points_;
    geometry_msgs::PoseArray pin_points_world_;
    nav_msgs::OccupancyGrid::ConstPtr global_costmap_;
    nav_msgs::OccupancyGrid::ConstPtr local_costmap_;
    geometry_msgs::PoseStamped current_pose_;
    tf::TransformListener tf_listener_;

    void pinPointCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
        if (!global_costmap_) {
            ROS_WARN("Global costmap not received yet.");
            return;
        }

        pin_points_.poses.clear();
        pin_points_world_.poses.clear();

        for (const auto& pose : msg->poses) {
            geometry_msgs::Pose transformed_pose = pose;
            double wx, wy;

            mapToWorld(global_costmap_, pose.position.x, pose.position.y, wx, wy);

            transformed_pose.position.x = wx;
            transformed_pose.position.y = wy;

            pin_points_.poses.push_back(pose);
            pin_points_world_.poses.push_back(transformed_pose);
        }

        ROS_INFO("Received and transformed pin points.");
    }

    void globalCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        global_costmap_ = msg;
    }

    void localCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        local_costmap_ = msg;
    }

    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        current_pose_.pose = msg->pose.pose;
        current_pose_.header = msg->header;
        ROS_INFO("Received AMCL pose.");
    }

    void checkForReplanning() {
        if (!global_costmap_ || !local_costmap_) {
            ROS_WARN("Costmaps not received yet.");
            trigger_replanning_ = false;
            return;
        }

        if (pin_points_world_.poses.empty()) {
            ROS_WARN("Pin points not received yet.");
            trigger_replanning_ = false;
            return;
        }

        for (int i = pin_points_world_.poses.size() - 1; i >= 0; --i) {
            double mx, my;
            worldToMap(local_costmap_, pin_points_world_.poses[i].position.x, pin_points_world_.poses[i].position.y, mx, my);

            // Check local costmap boundary
            if (mx < 0 || mx >= local_costmap_->info.width || my < 0 || my >= local_costmap_->info.height) {
                ROS_WARN("Pin point[%d] is out of local costmap bounds, skipping this point.", i);
                continue;
            }

            // Check if the point is in an obstacle or inflation layer in the local costmap
            int index = static_cast<int>(my) * local_costmap_->info.width + static_cast<int>(mx);
            ROS_INFO("Cost value at index [%d]: %d", index, local_costmap_->data[index]);
            if (local_costmap_->data[index] >= Constant::OBSTACLE_INFLATION_INIT) {
                ROS_WARN("Pin point[%d] is in an obstacle or inflation layer, skipping this point.", i);
                continue;
            }

            // Check the line to pin point is free in local costmap
            if (isPathClear(current_pose_.pose, pin_points_world_.poses[i], local_costmap_)) {
                // Check the line to pin point is not free in global costmap
                if (!isPathClear(current_pose_.pose, pin_points_world_.poses[i], global_costmap_)) {
                    ROS_INFO("Triggering replanning due to detected obstacle.");
                    trigger_replanning_ = true;
                    publishLineMarker(current_pose_.pose, pin_points_world_.poses[i]);
                    return;
                }
            }
        }

        ROS_INFO("No need for replanning.");
        trigger_replanning_ = false;
    }


    bool isPathClear(const geometry_msgs::Pose& start, const geometry_msgs::Pose& end, const nav_msgs::OccupancyGrid::ConstPtr& costmap) {
        int width = costmap->info.width;
        int height = costmap->info.height;
        double resolution = costmap->info.resolution;
        double origin_x = costmap->info.origin.position.x;
        double origin_y = costmap->info.origin.position.y;

        int start_x = static_cast<int>((start.position.x - origin_x) / resolution);
        int start_y = static_cast<int>((start.position.y - origin_y) / resolution);
        int end_x = static_cast<int>((end.position.x - origin_x) / resolution);
        int end_y = static_cast<int>((end.position.y - origin_y) / resolution);

        int dx = abs(end_x - start_x);
        int dy = abs(end_y - start_y);
        int sx = start_x < end_x ? 1 : -1;
        int sy = start_y < end_y ? 1 : -1;
        int err = dx - dy;

        while (start_x != end_x || start_y != end_y) {
            int index = start_y * width + start_x;
            if (index >= 0 && index < static_cast<int>(costmap->data.size())) {
                if (costmap->data[index] > 0) {
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

    void publishLineMarker(const geometry_msgs::Pose& start, const geometry_msgs::Pose& end) {
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "replanning_trigger";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;

        line_marker.scale.x = 0.05;
        line_marker.color.r = 1.0;
        line_marker.color.g = 0.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 1.0;

        geometry_msgs::Point p_start, p_end;

        p_start.x = start.position.x;
        p_start.y = start.position.y;
        p_start.z = start.position.z;

        p_end.x = end.position.x;
        p_end.y = end.position.y;
        p_end.z = end.position.z;

        line_marker.points.push_back(p_start);
        line_marker.points.push_back(p_end);

        line_pub_.publish(line_marker);
    }

    void publishAllPinPoints() {
        visualization_msgs::Marker points_marker;
        points_marker.header.frame_id = "map";
        points_marker.header.stamp = ros::Time::now();
        points_marker.ns = "pin_points";
        points_marker.id = 1;
        points_marker.type = visualization_msgs::Marker::POINTS;
        points_marker.action = visualization_msgs::Marker::ADD;

        points_marker.scale.x = 0.5;
        points_marker.scale.y = 0.5;
        points_marker.color.r = 0.0;
        points_marker.color.g = 1.0;
        points_marker.color.b = 0.0;
        points_marker.color.a = 1.0;

        for (const auto& pose : pin_points_world_.poses) {
            geometry_msgs::Point p;

            p.x = pose.position.x;
            p.y = pose.position.y;
            p.z = pose.position.z;

            points_marker.points.push_back(p);
        }

        pin_points_pub_.publish(points_marker);
    }

    void worldToMap(const nav_msgs::OccupancyGrid::ConstPtr& costmap, double wx, double wy, double& mx, double& my) {
        double origin_x = costmap->info.origin.position.x;
        double origin_y = costmap->info.origin.position.y;
        double resolution = costmap->info.resolution;

        mx = (wx - origin_x) / resolution;
        my = (wy - origin_y) / resolution;
    }

    void mapToWorld(const nav_msgs::OccupancyGrid::ConstPtr& costmap, double mx, double my, double& wx, double& wy) {
        double origin_x = costmap->info.origin.position.x;
        double origin_y = costmap->info.origin.position.y;
        double resolution = costmap->info.resolution;

        wx = mx * resolution + origin_x;
        wy = my * resolution + origin_y;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "replanning_trigger_node");
    ReplanningTriggerNode node;
    node.run();
    return 0;
}
