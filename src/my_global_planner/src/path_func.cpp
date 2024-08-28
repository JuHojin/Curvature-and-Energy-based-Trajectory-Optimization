// path_func.cpp
#include "my_global_planner.h"
#include "path_func.h"
#include "boundaries.h"
#include "constant.h"

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <tf/tf.h>

namespace my_global_planner {

void updateOrientatoin(std::vector<geometry_msgs::PoseStamped>& plan) {
    if (plan.size() < 2) {
        ROS_WARN("Fail updating orientation: at least two points are required.");
        return;
    }

    for (size_t i = 0; i < plan.size(); ++i) {
        double dx, dy, yaw;

        if (i == 0) {
            // For start point
            dx = plan[i + 1].pose.position.x - plan[i].pose.position.x;
            dy = plan[i + 1].pose.position.y - plan[i].pose.position.y;
        } else if (i == plan.size() - 1) {
            // For goal point
            dx = plan[i].pose.position.x - plan[i - 1].pose.position.x;
            dy = plan[i].pose.position.y - plan[i - 1].pose.position.y;
        } else {
            // For intermediate points
            dx = plan[i + 1].pose.position.x - plan[i - 1].pose.position.x;
            dy = plan[i + 1].pose.position.y - plan[i - 1].pose.position.y;
        }
        yaw = std::atan2(dy, dx);

        plan[i].pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    }
}

bool isClearLine(geometry_msgs::PoseStamped& start_pose, geometry_msgs::PoseStamped& end_pose) {
    //  Bresenham's line algorithm
    int start_x = start_pose.pose.position.x;
    int start_y = start_pose.pose.position.y;
    int dx = end_pose.pose.position.x - start_x;
    int dy = end_pose.pose.position.y - start_y;
    int abs_dx = abs(dx);
    int abs_dy = abs(dy);
    int x_inc = (dx > 0) ? 1 : -1;
    int y_inc = (dy > 0) ? 1 : -1;
    int n = 1 + abs_dx + abs_dy;
    int x_error = abs_dx - abs_dy;

    abs_dx *= 2;
    abs_dy *= 2;

    for (; n > 0; --n) {
        if (costmap->getCost(start_x, start_y) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
            return false;

        if (x_error > 0) {
            start_x += x_inc;
            x_error -= abs_dy;
        } else {
            start_y += y_inc;
            x_error += abs_dx;
        }
    }
    return true;
}

std::vector<geometry_msgs::PoseStamped> postSmooth (std::vector<geometry_msgs::PoseStamped>& plan) {
    int i = 0;
    std::vector<geometry_msgs::PoseStamped> new_plan;
    new_plan.push_back(plan[0]);

    while (i < plan.size() - 1) {
        ++i;
        if (!isClearLine(new_plan.back(), plan[i]))
            new_plan.push_back(plan[i-1]);
    
    }
    new_plan.push_back(plan[i]);
    new_plan.push_back(plan.back());

    updateOrientatoin(new_plan);

    return new_plan;
}

std::vector<geometry_msgs::PoseStamped> uniformSpacing(const std::vector<geometry_msgs::PoseStamped>& plan) {
    if (plan.empty()) {
        ROS_WARN("Uniform spacing failed: Path is empty.");
        return plan;
    }

    std::vector<geometry_msgs::PoseStamped> new_plan;
    new_plan.push_back(plan[0]);

    for (size_t i = 1; i < plan.size(); ++i) {
        geometry_msgs::PoseStamped current = new_plan.back();
        const geometry_msgs::PoseStamped& next = plan[i];
        double dist = std::hypot(next.pose.position.x - current.pose.position.x, next.pose.position.y - current.pose.position.y);

        while (dist > Constant::UNIFORM_STEP) {
            double dx = next.pose.position.x - current.pose.position.x;
            double dy = next.pose.position.y - current.pose.position.y;
            double length = std::hypot(dx, dy);
            
            // Normalize
            dx /= length;
            dy /= length;

            current.pose.position.x += dx * Constant::UNIFORM_STEP;
            current.pose.position.y += dy * Constant::UNIFORM_STEP;

            new_plan.push_back(current);
            
            // dist can be calculated using direction
            dist = std::hypot(next.pose.position.x - current.pose.position.x, next.pose.position.y - current.pose.position.y);
        }
        // Include corner point
        new_plan.push_back(next);
    }
    // Uncomment due to including corner point (end point)
    // new_plan.push_back(plan.back());
    updateOrientatoin(new_plan);

    return new_plan;
}

std::vector<geometry_msgs::PoseStamped> adaptiveSpacing(const std::vector<geometry_msgs::PoseStamped>& plan) {

}

} // namespace my_global_planner