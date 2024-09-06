// path_func.cpp
#include <ros/ros.h>
#include <tf/tf.h>
#include "my_global_planner.h"
#include "path_func.h"
#include "boundaries.h"
#include "constant.h"

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

double getTurningRadius(const Eigen::Vector2d& pt1, const Eigen::Vector2d& pt2, const Eigen::Vector2d& pt3) {
    Eigen::Vector2d center_point = getCircumcenter(pt1, pt2, pt3);

    if (!std::isfinite(center_point.x()) || !std::isfinite(center_point.y())) {
        return std::numeric_limits<double>::infinity();
    }

    double R = (pt2 - center_point).norm();
    return R;
}

std::tuple<Eigen::VectorXd, double, double> getCurvatureSquareMaxUsingTurningRadius(const std::vector<Eigen::Vector2d>& path) {
    int numPoints = path.size();
    Eigen::VectorXd curvature(numPoints);
    curvature.setZero();
    double curvature_square_sum = 0.0;
    double max_curvature = 0.0;

    for (int j = 1; j < numPoints - 1; ++j) {
        double R = getTurningRadius(path[j - 1], path[j], path[j + 1]);
        curvature[j] = 1.0 / R;
        curvature_square_sum += std::pow(curvature[j], 2);
        if (curvature[j] > max_curvature) {
            max_curvature = curvature[j];
        }
    }
    
    return {curvature, curvature_square_sum, max_curvature};
}

Eigen::VectorXd getCurvatureUsingTurningRadius(const std::vector<Eigen::Vector2d>& path) {
    int numPoints = path.size();
    Eigen::VectorXd curvature(numPoints);
    curvature.setZero();

    for (int j = 1; j < numPoints - 1; ++j) {
        double R = getTurningRadius(path[j - 1], path[j], path[j + 1]);
        curvature[j] = 1.0 / R;
    }
    
    return curvature;
}

Eigen::VectorXd getCurvatureUsingTurningRadius(const nav_msgs::Path& path) {
    int numPoints = path.poses.size(); // Use .poses.size() to get the number of points in the path

    Eigen::VectorXd curvature(numPoints);
    curvature.setZero();

    for (int j = 1; j < numPoints - 1; ++j) {
        // Extract the previous, current, and next points from the path
        Eigen::Vector2d prev_point(path.poses[j - 1].pose.position.x, path.poses[j - 1].pose.position.y);
        Eigen::Vector2d current_point(path.poses[j].pose.position.x, path.poses[j].pose.position.y);
        Eigen::Vector2d next_point(path.poses[j + 1].pose.position.x, path.poses[j + 1].pose.position.y);

        // Calculate the turning radius
        double R = getTurningRadius(prev_point, current_point, next_point);

        // Calculate the curvature (1 / radius)
        curvature[j] = 1.0 / R;
    }

    return curvature;
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
            // ROS_INFO("current (%f, %f)", current.pose.position.x, current.pose.position.y);
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

std::vector<Eigen::Vector2d> adaptiveSpacing(const std::vector<Eigen::Vector2d>& path) {
    std::vector<Eigen::Vector2d> new_path;
    new_path.push_back(path[0]);

    for (size_t i = 1; i < path.size(); ++i) {
        Eigen::Vector2d start = new_path.back();
        Eigen::Vector2d end = path[i];
        double dist = (end - start).norm();

        double min_obstacle_distance = get_min_obstacle_distance(start, Constant::ADAPTIVE_MAX_STEP);

        // Determine sampling spacing
        double new_spacing;
        if (min_obstacle_distance != -1 && min_obstacle_distance < Constant::ADAPTIVE_MAX_STEP) {
            if (min_obstacle_distance < Constant::ADAPTIVE_MIN_STEP) {
                new_spacing = Constant::ADAPTIVE_MIN_STEP;
            } else {
                new_spacing = min_obstacle_distance;
            }
        } else {
            new_spacing = Constant::ADAPTIVE_MAX_STEP;
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
                if (cost >= costmap_2d::LETHAL_OBSTACLE) {
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