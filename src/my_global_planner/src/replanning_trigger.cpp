// replanning_trigger.cpp
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "constant.h"

class ReplanningTriggerNode {
public:
    ReplanningTriggerNode() {
        trigger_pub_ = nh_.advertise<std_msgs::Bool>("/move_base/MyGlobalPlanner/replanning_trigger", 1);
        pin_point_list_sub_ = nh_.subscribe("/move_base/MyGlobalPlanner/pin_point_list", 1, &ReplanningTriggerNode::pinPointCallback, this);
        pin_point_indices_sub_ = nh_.subscribe("/move_base/MyGlobalPlanner/pin_point_indices", 1, &ReplanningTriggerNode::pinPointIndicesCallback, this);
        global_costmap_sub_ = nh_.subscribe("/move_base/global_costmap/costmap", 1, &ReplanningTriggerNode::globalCostmapCallback, this);
        local_costmap_sub_ = nh_.subscribe("/move_base/local_costmap/costmap", 1, &ReplanningTriggerNode::localCostmapCallback, this);
        amcl_pose_sub_ = nh_.subscribe("/amcl_pose", 1, &ReplanningTriggerNode::amclPoseCallback, this);
        final_plan_sub_ = nh_.subscribe("/move_base/MyGlobalPlanner/plan/final", 1, &ReplanningTriggerNode::finalPlanCallback, this);
        line_pub_ = nh_.advertise<visualization_msgs::Marker>("/replanning_trigger/line_marker", 1);
        pin_points_pub_ = nh_.advertise<visualization_msgs::Marker>("/replanning_trigger/pin_points_marker", 1);

        start_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/replanning_trigger/start_point", 1);
        ahead_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/replanning_trigger/ahead_point", 1);

        global_costmap_ = nullptr;
        local_costmap_ = nullptr;
    }

    void run() {
        ros::Rate rate(1.0);
        while (ros::ok()) {
            publishAllPinPoints();
            updateCurrentIndex();
            updateReplanningIndex();
            updateStartAndAheadIndex();
            ROS_WARN("replanning idx: %d", replanning_idx_);
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
    ros::Subscriber pin_point_list_sub_;
    ros::Subscriber pin_point_indices_sub_;
    ros::Publisher start_point_pub_;
    ros::Publisher ahead_point_pub_;
    ros::Subscriber global_costmap_sub_;
    ros::Subscriber local_costmap_sub_;
    ros::Subscriber amcl_pose_sub_;
    ros::Subscriber final_plan_sub_;

    bool trigger_replanning_ = false;
    int replanning_idx_ = 1;
    int current_idx_;
    int start_idx_;
    int ahead_idx_;
    geometry_msgs::PoseArray pin_points_;
    geometry_msgs::PoseArray pin_points_world_;
    std::vector<int> pin_point_indices_;
    nav_msgs::OccupancyGrid::ConstPtr global_costmap_;
    nav_msgs::OccupancyGrid::ConstPtr local_costmap_;
    geometry_msgs::PointStamped current_pose_;
    nav_msgs::Path final_plan_;
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

    void pinPointIndicesCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
        pin_point_indices_ = msg->data;
        std::reverse(pin_point_indices_.begin(), pin_point_indices_.end());
        ROS_INFO("Received and reversed pin point indices.");
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
        ROS_INFO("Received AMCL pose.");
    }

    void finalPlanCallback(const nav_msgs::Path::ConstPtr& msg) {
        final_plan_ = *msg;
        replanning_idx_ = 1;
        ROS_INFO("Received final plan.");
    }


    void updateCurrentIndex() {
        if (final_plan_.poses.empty()) {
            ROS_WARN("Final plan is empty, cannot find the nearest point.");
            current_idx_ = -1;
            return;
        }

        int nearest_index = -1;
        double min_distance = std::numeric_limits<double>::max();

        double current_x = current_pose_.point.x;
        double current_y = current_pose_.point.y;

        for (int i = 0; i < final_plan_.poses.size(); ++i) {
            double plan_x = final_plan_.poses[i].pose.position.x;
            double plan_y = final_plan_.poses[i].pose.position.y;

            double distance = std::sqrt(std::pow(plan_x - current_x, 2) + std::pow(plan_y - current_y, 2));

            if (distance < min_distance) {
                min_distance = distance;
                nearest_index = i;
            }
        }

        current_idx_ = nearest_index;
    }

    void updateStartAndAheadIndex() {
        double velocity = 1.0;
        double time = 2.0;

        start_idx_ = current_idx_ + static_cast<int>(velocity * time);
        ahead_idx_ = start_idx_ + static_cast<int>(velocity * time);

        publishStartAndAheadPoints();
    }

    void publishStartAndAheadPoints() {
        if (start_idx_ >= 0 && start_idx_ < final_plan_.poses.size()) {
            geometry_msgs::PointStamped start_point;
            start_point.header.frame_id = "map";
            start_point.header.stamp = ros::Time::now();
            start_point.point = final_plan_.poses[start_idx_].pose.position;

            start_point_pub_.publish(start_point);
        } else {
            ROS_WARN("Start index out of bounds, cannot publish start point.");
        }

        if (ahead_idx_ >= 0 && ahead_idx_ < final_plan_.poses.size()) {
            geometry_msgs::PointStamped ahead_point;
            ahead_point.header.frame_id = "map";
            ahead_point.header.stamp = ros::Time::now();
            ahead_point.point = final_plan_.poses[ahead_idx_].pose.position;

            ahead_point_pub_.publish(ahead_point);
        } else {
            ROS_WARN("Ahead index out of bounds, cannot publish ahead point.");
        }
    }

    void updateReplanningIndex() {
        if (current_idx_ == -1) {
            ROS_WARN("Unable to update replanning index because the nearest point index could not be found.");
            return;
        }
        
        // Debug in simulation
        // std::stringstream ss;
        // ss << "Pin point indices: ";
        // for (size_t i = 0; i < pin_point_indices_.size(); ++i) {
        //     ss << pin_point_indices_[i];
        //     if (i < pin_point_indices_.size() - 1) {
        //         ss << ", ";
        //     }
        // }
        // ROS_INFO("%s", ss.str().c_str());
        
        if (!(pin_point_indices_.empty()) && current_idx_ >= pin_point_indices_.back()) {
            pin_point_indices_.pop_back();
            ++replanning_idx_;
            if (replanning_idx_ >= pin_points_.poses.size()) {
                replanning_idx_ = pin_points_.poses.size() - 1;
            }
        }

        if (replanning_idx_ < pin_points_world_.poses.size()) {
            geometry_msgs::Pose point_to_publish = pin_points_world_.poses[replanning_idx_];

            // Create a red marker for the point
            visualization_msgs::Marker red_point_marker;
            red_point_marker.header.frame_id = "map";
            red_point_marker.header.stamp = ros::Time::now();
            red_point_marker.ns = "replanning_trigger";
            red_point_marker.id = replanning_idx_;
            red_point_marker.type = visualization_msgs::Marker::SPHERE;
            red_point_marker.action = visualization_msgs::Marker::ADD;

            red_point_marker.pose.position.x = point_to_publish.position.x;
            red_point_marker.pose.position.y = point_to_publish.position.y;
            red_point_marker.pose.position.z = point_to_publish.position.z;
            red_point_marker.pose.orientation.w = 1.0;

            red_point_marker.scale.x = 0.3;
            red_point_marker.scale.y = 0.3;
            red_point_marker.scale.z = 0.3;

            red_point_marker.color.r = 1.0;
            red_point_marker.color.g = 0.0;
            red_point_marker.color.b = 0.0;
            red_point_marker.color.a = 1.0;

            line_pub_.publish(red_point_marker);
        } else {
            ROS_WARN("Replanning index out of bounds, cannot publish marker.");
        }
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

        for (int i = pin_points_world_.poses.size() - 1; i >= replanning_idx_; --i) {
            geometry_msgs::PointStamped global_point, local_point;
            global_point.point.x = pin_points_world_.poses[i].position.x;
            global_point.point.y = pin_points_world_.poses[i].position.y;
            global_point.point.z = 0;
            global_point.header.frame_id = "map";
            global_point.header.stamp = ros::Time(0);

            try {
                tf_listener_.transformPoint("odom", global_point, local_point);

                double pin_point_x_odom = local_point.point.x;
                double pin_point_y_odom = local_point.point.y;

                float local_costmap_x = local_costmap_->info.origin.position.x;
                float local_costmap_y = local_costmap_->info.origin.position.y;

                double mx = (pin_point_x_odom - local_costmap_x) / local_costmap_->info.resolution;
                double my = (pin_point_y_odom - local_costmap_y) / local_costmap_->info.resolution;

                if (mx < 0 || my >= local_costmap_->info.width || mx < 0 || my >= local_costmap_->info.height) {
                    ROS_INFO("Pin point[%d] is (%f, %f) out of local costmap bounds (%d, %d), skipping this point.", i, mx, my, local_costmap_->info.width, local_costmap_->info.height);
                    continue;
                }

                int index = static_cast<int>(my) * local_costmap_->info.width + static_cast<int>(mx);

                if (local_costmap_->data[index] != 0) {
                    ROS_INFO("Pin point[%d] is in an obstacle, inflation layer, or unknown region, skipping this point.", i);
                    continue;
                }

                if (isPathClear(final_plan_.poses[ahead_idx_].pose.position, local_point.point, local_costmap_)) {
                    if (!isPathClear(final_plan_.poses[ahead_idx_].pose.position, local_point.point, global_costmap_)) {
                        ROS_INFO("\nTrigger replanning!\n");
                        trigger_replanning_ = true;
                        publishLineMarker(final_plan_.poses[ahead_idx_].pose.position, pin_points_world_.poses[i].position);
                        return;
                    }
                }

            } catch (tf2::TransformException& ex) {
                ROS_WARN("Transform error: %s", ex.what());
                continue;
            }
        }

        ROS_INFO("No need for replanning.");
        trigger_replanning_ = false;
    }

    bool isPathClear(const geometry_msgs::Point& start, const geometry_msgs::Point& end, const nav_msgs::OccupancyGrid::ConstPtr& costmap) {
        int width = costmap->info.width;
        int height = costmap->info.height;
        double resolution = costmap->info.resolution;
        double origin_x = costmap->info.origin.position.x;
        double origin_y = costmap->info.origin.position.y;

        int start_x = static_cast<int>((start.x - origin_x) / resolution);
        int start_y = static_cast<int>((start.y - origin_y) / resolution);
        int end_x = static_cast<int>((end.x - origin_x) / resolution);
        int end_y = static_cast<int>((end.y - origin_y) / resolution);


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

    void publishLineMarker(const geometry_msgs::Point& start, const geometry_msgs::Point& end) {
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

        p_start.x = start.x;
        p_start.y = start.y;
        p_start.z = start.z;

        p_end.x = end.x;
        p_end.y = end.y;
        p_end.z = end.z;

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

        points_marker.scale.x = 0.1;
        points_marker.scale.y = 0.1;
        points_marker.color.r = 0.0;
        points_marker.color.g = 1.0;
        points_marker.color.b = 0.0;
        points_marker.color.a = 1.0;

        for (const auto& pose : pin_points_world_.poses) {
            points_marker.points.push_back(pose.position);
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
