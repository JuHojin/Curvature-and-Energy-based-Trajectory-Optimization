// path_func.cpp
#include <ros/ros.h>
#include <tf/tf.h>
#include "my_global_planner.h"
#include "path_func.h"
#include "boundaries.h"
#include "constant.h"

namespace my_global_planner {

// ========================================
// Update Path Information
// ========================================
void updateOrientation(std::vector<geometry_msgs::PoseStamped>& plan) {
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

// ========================================
// Post Smoothing
// ========================================
std::vector<geometry_msgs::PoseStamped> postSmooth (std::vector<geometry_msgs::PoseStamped>& plan) {
    int i = 0;
    std::vector<geometry_msgs::PoseStamped> new_plan;
    new_plan.push_back(plan[0]);

    while (i < plan.size()) {
        ++i;
        if (!isClearLine(new_plan.back(), plan[i])) {
            new_plan.push_back(plan[i-1]);
        }
    
    }
    new_plan.push_back(plan.back());

    updateOrientation(new_plan);
    
    return new_plan;
}

// ========================================
// Path Spacing
// ========================================
std::vector<geometry_msgs::PoseStamped> uniformSpacing(const std::vector<geometry_msgs::PoseStamped>& plan, double step_size) {
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
        // if (dist < step_size) {
        //     ++i;
        //     continue;
        // }

        while (dist > step_size) {
            double dx = next.pose.position.x - current.pose.position.x;
            double dy = next.pose.position.y - current.pose.position.y;
            double length = std::hypot(dx, dy);
            
            // Normalize
            dx /= length;
            dy /= length;

            current.pose.position.x += dx * step_size;
            current.pose.position.y += dy * step_size;

            new_plan.push_back(current);
            // ROS_INFO("current (%f, %f)", current.pose.position.x, current.pose.position.y);
            dist = std::hypot(next.pose.position.x - current.pose.position.x, next.pose.position.y - current.pose.position.y);
        }
        // Include corner point
        // new_plan.push_back(next);

        ++i;
    }
    // Uncomment due to including corner point (end point)
    new_plan.push_back(plan.back());
    
    updateOrientation(new_plan);

    return new_plan;
}

std::vector<Eigen::Vector2d> uniformSpacing(const std::vector<Eigen::Vector2d>& path, double step_size) {
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

        // if (dist < step_size) {
        //     ++i;
        //     continue;
        // }

        while (dist > step_size) {
            Eigen::Vector2d direction = (next - current).normalized();
            current += direction * step_size;

            new_path.push_back(current);
            
            dist = (next - current).norm();
        }
        // new_path.push_back(next);
        ++i;
    }
    new_path.push_back(path.back());

    return new_path;
}

std::vector<Eigen::Vector2d> adaptiveSpacing(const std::vector<Eigen::Vector2d>& path, double min_step_size, double max_step_size) {
    std::vector<Eigen::Vector2d> new_path;
    new_path.push_back(path[0]);

    for (size_t i = 1; i < path.size(); ++i) {
        Eigen::Vector2d start = new_path.back();
        Eigen::Vector2d end = path[i];
        double dist = (end - start).norm();

        double min_obstacle_distance = get_min_obstacle_distance(start, max_step_size);

        // Determine sampling spacing
        double new_spacing;
        if (min_obstacle_distance != -1 && min_obstacle_distance < max_step_size) {
            if (min_obstacle_distance < min_step_size) {
                new_spacing = min_step_size;
            } else {
                new_spacing = min_obstacle_distance;
            }
        } else {
            new_spacing = max_step_size;
        }

        // std::cout << "Min obstacle distance: " << min_obstacle_distance << ", New spacing: " << new_spacing << std::endl;

        while (dist > new_spacing) {
            Eigen::Vector2d direction = (end - start).normalized();
            start = start + direction * new_spacing;
            new_path.push_back(start);
            dist = (end - start).norm();
        }
    }
    new_path.push_back(path.back());  // Ensure the last point is included

    return new_path;
}

// ========================================
// Obstacles Related Functions
// ========================================
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
        if (costmap->getCost(start_x, start_y) >= Constant::OBSTACLE_INFLATION_INIT)
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

double get_min_obstacle_distance(Eigen::Vector2d start, unsigned int searching_radius) {
    unsigned int start_x = static_cast<unsigned int>(start.x());
    unsigned int start_y = static_cast<unsigned int>(start.y());

    unsigned int costmap_width = costmap->getSizeInCellsX();
    unsigned int costmap_height = costmap->getSizeInCellsY();
    double resolution = costmap->getResolution();

    std::vector<double> distances;

    // Loop through the search area around the point
    for (int dx = -static_cast<int>(searching_radius); dx <= static_cast<int>(searching_radius); ++dx) {
        for (int dy = -static_cast<int>(searching_radius); dy <= static_cast<int>(searching_radius); ++dy) {
            int x = start_x + dx;
            int y = start_y + dy;

            // Check if the point is within the costmap bounds
            if (x >= 0 && x < costmap_width && y >= 0 && y < costmap_height) {
                unsigned char cost = costmap->getCost(x, y);
                
                // Check if the cell is an obstacle (considering a threshold)
                if (cost >= Constant::OBSTACLE_INFLATION) {
                    double dist = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
                    if (dist <= searching_radius) {
                        distances.push_back(dist);
                    }
                }
            }
        }
    }

    if (distances.empty()) {
        return -1;
    } else {
        return *std::min_element(distances.begin(), distances.end());
    }
}

// ========================================
// Velocity Profile
// ========================================
std::pair<Eigen::VectorXd, Eigen::MatrixXd> getVelocityProfile(const Eigen::VectorXd& curvature, double step_size, double init_velocity, double max_vel, double max_ax, double max_ay) {

    size_t len_path = curvature.size();

    // Initialize the maximum velocity array with the constant max_vel for all path points
    Eigen::VectorXd max_vel_list = Eigen::VectorXd::Ones(len_path) * max_vel;
    Eigen::VectorXd lat_acc = Eigen::VectorXd::Zero(len_path);

    // Compute the lateral acceleration constraints
    for (size_t i = 0; i < len_path; ++i) {
        double k = curvature(i);
        double v = std::sqrt(max_ay / std::abs(k));
        // Cap the velocity at max_vel + 1 for smooth plot (does not affect the velocity profile)
        v = std::min(v, max_vel + 1);
        lat_acc(i) = v;
        if (v < max_vel) {
            ROS_INFO("\n[%d] k: %f, vy: %f", i, k, v);
        }
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
        double next_v = std::sqrt(v * v + 2 * max_ax * step_size);
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
        double next_v = std::sqrt(v * v + 2 * max_ax * step_size);
        if (next_v < velocity_profile(i)) {
            velocity_profile(i) = next_v;
            v = next_v;
        } else {
            v = velocity_profile(i);
        }
    }

    return std::make_pair(velocity_profile, velocity_constraints);
}

// ========================================
// Curvatures
// ========================================
double calculateDistance(const Eigen::Vector2d& pt1, const Eigen::Vector2d& pt2) {
    return (pt2 - pt1).norm();
}

Eigen::Vector2d getCircumcenter(const Eigen::Vector2d& pt1, const Eigen::Vector2d& pt2, const Eigen::Vector2d& pt3) {
    double ax = pt1.x(), ay = pt1.y();
    double bx = pt2.x(), by = pt2.y();
    double cx = pt3.x(), cy = pt3.y();

    double d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
    if (d == 0) {
        return Eigen::Vector2d(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    } else {
        double ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d;
        double uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d;
        return Eigen::Vector2d(ux, uy);
    }
}

Eigen::VectorXd getCurvatureUsingTurningRadius(const std::vector<Eigen::Vector2d>& path) {
    int n = path.size();
    Eigen::VectorXd curvature(n);

    for (int i = 1; i < n - 1; ++i) {
        Eigen::Vector2d center_point = getCircumcenter(path[i - 1], path[i], path[i + 1]);

        if (!std::isfinite(center_point.x()) || !std::isfinite(center_point.y())) {
            curvature(i) = 0.0;
        } else {
            double R = calculateDistance(path[i], center_point);
            if (R != 0.0) {
                curvature(i) = 1.0 / R;
            } else {
                curvature(i) = 0.0;
            }
        }
    }

    curvature(0) = 0.0;
    curvature(n - 1) = 0.0;

    return curvature;
}

Eigen::VectorXd getCurvatureUsingTurningRadius(const nav_msgs::Path& path_msg) {
    int n = path_msg.poses.size();
    Eigen::VectorXd curvature(n);

    std::vector<Eigen::Vector2d> path;
    for (const auto& pose : path_msg.poses) {
        double x = pose.pose.position.x;
        double y = pose.pose.position.y;
        path.emplace_back(x, y);
    }

    for (int i = 1; i < n - 1; ++i) {
        Eigen::Vector2d center_point = getCircumcenter(path[i - 1], path[i], path[i + 1]);

        if (!std::isfinite(center_point.x()) || !std::isfinite(center_point.y())) {
            curvature(i) = 0.0;
        } else {
            double R = calculateDistance(path[i], center_point);
            if (R != 0.0) {
                curvature(i) = 1.0 / R;
            } else {
                curvature(i) = 0.0;
            }
        }
    }

    curvature(0) = 0.0;
    curvature(n - 1) = 0.0;

    return curvature;
}

Eigen::VectorXd getCurvatureUsingTurningRadius(const std::vector<geometry_msgs::PoseStamped>& plan) {
    int n = plan.size();
    Eigen::VectorXd curvature(n);

    std::vector<Eigen::Vector2d> path;
    for (const auto& pose : plan) {
        double x = pose.pose.position.x;
        double y = pose.pose.position.y;
        path.emplace_back(x, y);
    }

    for (int i = 1; i < n - 1; ++i) {
        Eigen::Vector2d center_point = getCircumcenter(path[i - 1], path[i], path[i + 1]);

        if (!std::isfinite(center_point.x()) || !std::isfinite(center_point.y())) {
            curvature(i) = 0.0;
        } else {
            double R = calculateDistance(path[i], center_point);
            if (R != 0.0) {
                curvature(i) = 1.0 / R;
            } else {
                curvature(i) = 0.0;
            }
        }
    }

    curvature(0) = 0.0;
    curvature(n - 1) = 0.0;

    return curvature;
}

Eigen::VectorXd getCurvatureUsingFiniteDifference(const nav_msgs::Path& path_msg) {
    int n = path_msg.poses.size();
    Eigen::VectorXd curvature(n);

    std::vector<Eigen::Vector2d> path;
    for (const auto& pose : path_msg.poses) {
        double x = pose.pose.position.x;
        double y = pose.pose.position.y;
        path.emplace_back(x, y);
    }

    for (int i = 1; i < n - 1; ++i) {
        Eigen::Vector2d d1 = path[i] - path[i - 1];
        Eigen::Vector2d d2 = path[i + 1] - path[i];

        Eigen::Vector2d velocity = (d1 + d2) / 2.0;

        Eigen::Vector2d acceleration = d2 - d1;

        double numerator = velocity.x() * acceleration.y() - velocity.y() * acceleration.x();
        double denominator = std::pow(velocity.squaredNorm(), 1.5);

        if (denominator != 0.0) {
            curvature(i) = numerator / denominator;
        } else {
            curvature(i) = 0.0;
        }
    }

    curvature(0) = 0.0;
    curvature(n - 1) = 0.0;

    return curvature;
}

} // namespace my_global_planner