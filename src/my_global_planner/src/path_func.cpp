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
        if (costmap->getCost(start_x, start_y) >= Constant::OBSTACLE_INFLATION)
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

bool isClearLine(int start_x, int start_y, int end_x, int end_y) {
    // Bresenham's line algorithm
    int dx = end_x - start_x;
    int dy = end_y - start_y;
    int abs_dx = abs(dx);
    int abs_dy = abs(dy);
    int x_inc = (dx > 0) ? 1 : -1;
    int y_inc = (dy > 0) ? 1 : -1;
    int n = 1 + abs_dx + abs_dy;
    int x_error = abs_dx - abs_dy;

    abs_dx *= 2;
    abs_dy *= 2;

    for (; n > 0; --n) {
        if (costmap->getCost(start_x, start_y) >= Constant::OBSTACLE_INFLATION)
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

double getTurningRadius(const Eigen::Vector2d& pt1, const Eigen::Vector2d& pt2, const Eigen::Vector2d& pt3) {
    // Calculate turning radius based on three points
    double a = (pt2 - pt1).norm();
    double b = (pt3 - pt2).norm();
    double c = (pt3 - pt1).norm();

    double s = (a + b + c) / 2.0;
    double area = std::sqrt(s * (s - a) * (s - b) * (s - c));

    double radius = (a * b * c) / (4.0 * area);
    return radius;
}

void getCurvatureUsingTurningRadius(const std::vector<Eigen::Vector2d>& path, 
                                    std::vector<double>& curvature,
                                    double& curvature_square_sum,
                                    double& max_curvature) {
    int numPoints = path.size();
    curvature.resize(numPoints);
    curvature_square_sum = 0.0;
    max_curvature = 0.0;

    for (int j = 1; j < numPoints - 1; ++j) {
        double R = getTurningRadius(path[j - 1], path[j], path[j + 1]);
        curvature[j] = 1.0 / R;
        curvature_square_sum += std::pow(curvature[j], 2);
        if (curvature[j] > max_curvature) {
            max_curvature = curvature[j];
        }
    }
}

std::vector<geometry_msgs::PoseStamped> postSmooth (std::vector<geometry_msgs::PoseStamped>& plan) {
    int i = 0;
    std::vector<geometry_msgs::PoseStamped> new_plan;
    new_plan.push_back(plan[0]);

    while (i < plan.size()) {
        ++i;
        if (!isClearLine(new_plan.back(), plan[i]))
            new_plan.push_back(plan[i-1]);
    
    }
    // new_plan.push_back(plan[i]);
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

    size_t i = 1;
    while (i < plan.size()) {
        geometry_msgs::PoseStamped current = new_plan.back();
        const geometry_msgs::PoseStamped& next = plan[i];
        double dist = std::hypot(next.pose.position.x - current.pose.position.x, next.pose.position.y - current.pose.position.y);
        // if (dist < Constant::UNIFORM_STEP) {
        //     ++i;
        //     continue;
        // }

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
            ROS_INFO("current (%f, %f)", current.pose.position.x, current.pose.position.y);
            dist = std::hypot(next.pose.position.x - current.pose.position.x, next.pose.position.y - current.pose.position.y);
        }
        // Include corner point
        // new_plan.push_back(next);

        ++i;
    }
    // Uncomment due to including corner point (end point)
    new_plan.push_back(plan.back());
    
    updateOrientatoin(new_plan);

    return new_plan;
}

std::vector<Eigen::Vector2d> uniformSpacing(const std::vector<Eigen::Vector2d>& path) {
    if (path.empty()) {
        std::cerr << "Uniform spacing failed: Path is empty." << std::endl;
        return path;
    }

    std::vector<Eigen::Vector2d> new_path;
    new_path.push_back(path.front());

    size_t i = 1;
    while (i < path.size()) {
        Eigen::Vector2d current = new_path.back();
        const Eigen::Vector2d& next = path[i];
        double dist = (next - current).norm();

        // if (dist < Constant::UNIFORM_STEP) {
        //     ++i;
        //     continue;
        // }

        while (dist > Constant::UNIFORM_STEP) {
            Eigen::Vector2d direction = (next - current).normalized();
            current += direction * Constant::UNIFORM_STEP;

            new_path.push_back(current);
            
            dist = (next - current).norm();
        }
        // new_path.push_back(next);
        ++i;
    }
    new_path.push_back(path.back());

    return new_path;
}

std::vector<geometry_msgs::PoseStamped> adaptiveSpacing(const std::vector<geometry_msgs::PoseStamped>& plan) {

}

std::pair<Eigen::VectorXd, Eigen::MatrixXd> getVelocityProfile(const Eigen::VectorXd& curvature, double init_velocity, double max_vel, double max_ax, double max_ay) {

    size_t len_path = curvature.size();

    // Initialize the maximum velocity array with the constant max_vel for all path points
    Eigen::VectorXd max_vel_list = Eigen::VectorXd::Ones(len_path) * max_vel;
    Eigen::VectorXd lat_acc = Eigen::VectorXd::Zero(len_path);

    // Compute the lateral acceleration constraints
    for (size_t i = 0; i < len_path; ++i) {
        double k = curvature(i);
        if (k < 1e-7) {
            k = 1e-7;
        }
        double v = std::sqrt(max_ay / k);
        // Cap the velocity at max_vel + 1 for smooth plot (does not affect the velocity profile)
        v = std::min(v, max_vel + 1);
        lat_acc(i) = v;
    }

    Eigen::MatrixXd velocity_constraints(2, len_path);
    velocity_constraints.row(0) = max_vel_list;
    velocity_constraints.row(1) = lat_acc;

    // Compute the final velocity profile by taking the minimum of max velocity and lateral acceleration constraints
    Eigen::VectorXd velocity_profile = velocity_constraints.colwise().minCoeff();

    // Apply forward acceleration constraint
    double v = init_velocity;  // Initial velocity
    velocity_profile(0) = v;
    for (size_t i = 1; i < len_path; ++i) {
        double next_v = std::sqrt(v * v + 2 * max_ax * Constant::UNIFORM_STEP);
        if (next_v < velocity_profile(i)) {
            velocity_profile(i) = next_v;
            v = next_v;
        } else {
            v = velocity_profile(i);
        }
    }

    // Apply backward acceleration constraint
    v = 0;  // Final velocity
    velocity_profile(len_path - 1) = v;
    for (int i = len_path - 2; i >= 0; --i) {
        double next_v = std::sqrt(v * v + 2 * max_ax * Constant::UNIFORM_STEP);
        if (next_v < velocity_profile(i)) {
            velocity_profile(i) = next_v;
            v = next_v;
        } else {
            v = velocity_profile(i);
        }
    }

    return std::make_pair(velocity_profile, velocity_constraints);
}

} // namespace my_global_planner