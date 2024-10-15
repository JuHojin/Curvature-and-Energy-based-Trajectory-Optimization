// opt_curv.cpp
#pragma GCC push_options
#pragma GCC optimize ("O0")

#include <ros/ros.h>
#include <cmath>
#include <limits>
#include <tf/tf.h>
#include <numeric>
#include <std_msgs/Bool.h>
#include "my_global_planner.h"
#include "opt_curv.h"
#include "path_func.h"
#include "constant.h"

namespace my_global_planner {

// Publishers
ros::Publisher plan_publisher_bnd_L;
ros::Publisher plan_publisher_bnd_R;
ros::Publisher plan_publisher_bnd_L_replan;
ros::Publisher plan_publisher_bnd_R_replan;
ros::Publisher plan_publisher_opt;
ros::Publisher optimization_status_publisher;

// ========================================
// Main
// ========================================
std::vector<Eigen::Vector2d> optimize_min_curvature(std::vector<Eigen::Vector2d> path, const size_t max_iter, bool local, tf2_ros::Buffer& tf_buffer) {
    
    std::vector<Eigen::Vector2d> new_path = path;
    std::vector<double> travel_times;
    Boundaries prev_boundaries;
    bool success_flag;

    int qp_type;
    ros::NodeHandle nh("/");
    nh.param("move_base/QP_type", qp_type, 1);

    for (int i = 0; i < max_iter; ++i) {
        ROS_INFO("\n============ Opt. Step [%d] ============", i + 1);
        if (qp_type == 0) {
            ROS_INFO("Using uniformSpacing for optimization step %d", i + 1);
            path = uniformSpacing(new_path, Constant::UNIFORM_STEP);
        } else if (qp_type == 1) {
            ROS_INFO("Using adaptiveSpacing for optimization step %d", i + 1);
            path = adaptiveSpacing(new_path, Constant::ADAPTIVE_MIN_STEP, Constant::ADAPTIVE_MAX_STEP);
        } else {
            ROS_WARN("Invalid QP_type value: %d. Defaulting to adaptiveSpacing.", qp_type);
            path = adaptiveSpacing(new_path, Constant::ADAPTIVE_MIN_STEP, Constant::ADAPTIVE_MAX_STEP);
        }

        Eigen::MatrixXd A = getSystemMatrix(path);
        std::vector<Eigen::Vector2d> normal_vectors = getNormalVectors(path);

        // Boundaries
        Boundaries Boundaries = getBoundaries(path, normal_vectors, local);
        prev_boundaries = Boundaries;
        // for (size_t i = 0; i < Boundaries.left_bnd.distances.size(); ++i) {
        //     ROS_INFO("Distance[%zu]: %f", i, Boundaries.left_bnd.distances[i]);
        // }

        // Publish Boundary Lines
        // Normal planning
        if (!local){
            std::vector<geometry_msgs::PoseStamped> left_bnd = convertVector2dToPoseStamped(Boundaries.left_bnd.points);
            std::vector<geometry_msgs::PoseStamped> right_bnd = convertVector2dToPoseStamped(Boundaries.right_bnd.points);
            nav_msgs::Path left_bnd_world = producePathMapToWorld(left_bnd);
            nav_msgs::Path right_bnd_world = producePathMapToWorld(right_bnd);
            publishPath(plan_publisher_bnd_L, left_bnd_world);
            publishPath(plan_publisher_bnd_R, right_bnd_world);
        }
        // Replanning
        else {
            std::vector<Eigen::Vector2d> left = Boundaries.left_bnd.points;
            std::vector<Eigen::Vector2d> right = Boundaries.right_bnd.points;

            // 1. Local costmap (map) -> Local costmap (world)
            for (auto& point : left) {
                double local_wx = point.x() * local_costmap_msg->info.resolution + local_costmap_msg->info.origin.position.x;
                double local_wy = point.y() * local_costmap_msg->info.resolution + local_costmap_msg->info.origin.position.y;
                point.x() = local_wx;
                point.y() = local_wy;
            }

            for (auto& point : right) {
                double local_wx = point.x() * local_costmap_msg->info.resolution + local_costmap_msg->info.origin.position.x;
                double local_wy = point.y() * local_costmap_msg->info.resolution + local_costmap_msg->info.origin.position.y;
                point.x() = local_wx;
                point.y() = local_wy;
            }

            // 2. Local costmap (world) -> Global costmap (world) using TF (transform odom to map)
            for (auto& point : left) {
                geometry_msgs::PointStamped local_point, global_point;
                local_point.point.x = point.x();
                local_point.point.y = point.y();
                local_point.header.frame_id = "odom";
                local_point.header.stamp = ros::Time(0);
                
                try {
                    // Use tf2 to transform point from "odom" frame to "map" frame
                    geometry_msgs::TransformStamped transform_stamped = tf_buffer.lookupTransform("map", "odom", ros::Time(0), ros::Duration(1.0));
                    tf2::doTransform(local_point, global_point, transform_stamped);
                    point.x() = global_point.point.x;
                    point.y() = global_point.point.y;
                } catch (tf2::TransformException& ex) {
                    ROS_WARN("Transform error: %s", ex.what());
                }
            }

            for (auto& point : right) {
                geometry_msgs::PointStamped local_point, global_point;
                local_point.point.x = point.x();
                local_point.point.y = point.y();
                local_point.header.frame_id = "odom";
                local_point.header.stamp = ros::Time(0);
                
                try {
                    // Use tf2 to transform point from "odom" frame to "map" frame
                    geometry_msgs::TransformStamped transform_stamped = tf_buffer.lookupTransform("map", "odom", ros::Time(0), ros::Duration(1.0));
                    tf2::doTransform(local_point, global_point, transform_stamped);
                    point.x() = global_point.point.x;
                    point.y() = global_point.point.y;
                } catch (tf2::TransformException& ex) {
                    ROS_WARN("Transform error: %s", ex.what());
                }
            }

            // 3. Global costmap (world) -> Global costmap (map)
            for (auto& point : left) {
                double mx, my;
                if (convertWorldToMap(point.x(), point.y(), mx, my)) {
                    point.x() = mx;
                    point.y() = my;
                } else {
                    ROS_WARN("Point out of global costmap bounds.");
                }
            }

            for (auto& point : right) {
                double mx, my;
                if (convertWorldToMap(point.x(), point.y(), mx, my)) {
                    point.x() = mx;
                    point.y() = my;
                } else {
                    ROS_WARN("Point out of global costmap bounds.");
                }
            }

            // Convert the points to PoseStamped and publish
            std::vector<geometry_msgs::PoseStamped> left_bnd_plan = convertVector2dToPoseStamped(left);
            std::vector<geometry_msgs::PoseStamped> right_bnd_plan = convertVector2dToPoseStamped(right);
            nav_msgs::Path left_bnd_replan_world = producePathMapToWorld(left_bnd_plan);
            nav_msgs::Path right_bnd_replan_world = producePathMapToWorld(right_bnd_plan);
            publishPath(plan_publisher_bnd_L_replan, left_bnd_replan_world);
            publishPath(plan_publisher_bnd_R_replan, right_bnd_replan_world);
        }

        // Calculate start and end angles
        Eigen::Vector2d start_segment = path[1] - path[0];
        Eigen::Vector2d end_segment = path[path.size() - 1] - path[path.size() - 2];
        double start_angle = std::atan2(start_segment.y(), start_segment.x());
        double end_angle = std::atan2(end_segment.y(), end_segment.x());

        // Alpha calculation
        auto result = getAlphaMinCurvature(path, Boundaries.left_bnd.distances, Boundaries.right_bnd.distances, normal_vectors, A, Constant::CURVATURE_BOUND, start_angle, end_angle, Constant::GAMMA);
        Eigen::VectorXd alpha = result.first;
        int success = result.second;

        if (success == OSQP_SOLVED) {
            new_path = getPathMinCurvature(path, normal_vectors, alpha);
            std::vector<geometry_msgs::PoseStamped> new_path_tmp = convertVector2dToPoseStamped(new_path);
            nav_msgs::Path new_path_world = producePathMapToWorld(new_path_tmp);
            publishPath(plan_publisher_opt, new_path_world);
            
        } else {
            ROS_INFO("Optimization Failed at step [%d]", i + 1);
            new_path = path;
            // Reverting to the previous boundaries
            std::vector<geometry_msgs::PoseStamped> left_bnd_prev = convertVector2dToPoseStamped(prev_boundaries.left_bnd.points);
            std::vector<geometry_msgs::PoseStamped> right_bnd_prev = convertVector2dToPoseStamped(prev_boundaries.right_bnd.points);
            nav_msgs::Path left_bnd_prev_world = producePathMapToWorld(left_bnd_prev);
            nav_msgs::Path right_bnd_prev_world = producePathMapToWorld(right_bnd_prev);
            publishPath(plan_publisher_bnd_L, left_bnd_prev_world);
            publishPath(plan_publisher_bnd_R, right_bnd_prev_world);
            
            success_flag = false;
            break;
        }

        // TODO: uniform apply or not apply

        // Get curvature and velocity profile
        std::vector<Eigen::Vector2d> new_path_uniform = uniformSpacing(new_path, Constant::UNIFORM_STEP);
        Eigen::VectorXd curvature = getCurvatureUsingTurningRadius(new_path_uniform);
        auto velocity_result = getVelocityProfile(curvature, Constant::UNIFORM_STEP, Constant::INIT_VELOCITY, Constant::MAX_VEL, Constant::MAX_AX, Constant::MAX_AY);
        Eigen::VectorXd velocity_profile = velocity_result.first;

        // Travel time
        double travel_time = getTravelTime(new_path_uniform, velocity_profile);
        ROS_INFO("\nOpt. Step [%d] Expected Travel Time: %f", i + 1, travel_time);
        travel_times.push_back(travel_time);

        // Compare current travel time with previous iteration
        if (i >= 2 && (travel_time > travel_times[i - 1] || std::isnan(travel_time))) {
            ROS_INFO("Current travel time is longer than the previous one or is NaN. Reverting to path at step [%d]", i);
            // Revert to the previous path
            new_path = path; 
            // Revert to the previous boundaries
            std::vector<geometry_msgs::PoseStamped> left_bnd_prev = convertVector2dToPoseStamped(prev_boundaries.left_bnd.points);
            std::vector<geometry_msgs::PoseStamped> right_bnd_prev = convertVector2dToPoseStamped(prev_boundaries.right_bnd.points);
            nav_msgs::Path left_bnd_prev_world = producePathMapToWorld(left_bnd_prev);
            nav_msgs::Path right_bnd_prev_world = producePathMapToWorld(right_bnd_prev);
            publishPath(plan_publisher_bnd_L, left_bnd_prev_world);
            publishPath(plan_publisher_bnd_R, right_bnd_prev_world);

            success_flag = true;
            break;
        }
    }

    std_msgs::Bool status_msg;
    status_msg.data = success_flag;
    optimization_status_publisher.publish(status_msg);

    // the return path is adaptive spaced path
    return new_path;
}

// ========================================
// Path Geometry Extractors
// ========================================
Eigen::MatrixXd getSystemMatrix(const std::vector<Eigen::Vector2d>& path) {
    int num_points = path.size();
    int num_splines = num_points - 1;
    Eigen::VectorXd pointDist(num_splines);

    // Calculate distances between consecutive points
    for (int i = 0; i < num_splines; ++i) {
        pointDist(i) = (path[i + 1] - path[i]).norm();
    }

    Eigen::VectorXd scaling(num_splines - 1);
    for (int i = 0; i < num_splines - 1; ++i) {
        scaling(i) = pointDist(i) / pointDist(i + 1);
    }

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_splines * 4, num_splines * 4);

    // Base matrix for constraints, repeated in A
    Eigen::MatrixXd baseA(4, 8);
    baseA   <<  1, 0, 0, 0, 0, 0, 0, 0,
                1, 1, 1, 1, 0, 0, 0, 0,
                0, 1, 2, 3, 0, -1, 0, 0,
                0, 0, 2, 6, 0, 0, -2, 0;

    // Construct the system matrix with appropriate scaling for continuity constraints
    for (int i = 0; i < num_splines; ++i) {
        int j = i * 4;

        if (i < num_splines - 1) {
            A.block(j, j, 4, 8) = baseA;

            // Apply scaling factors to the derivative constraints
            A(j + 2, j + 5) *= scaling(i);
            A(j + 3, j + 6) *= std::pow(scaling(i), 2);
        } else {
            // End conditions for the spline (ensure the spline ends smoothly at the last point)
            A.block(j, j, 2, 4) = baseA.block(0, 0, 2, 4);
        }
    }

    // Starting point heading constraint
    A(num_splines * 4 - 2, 1) = 1;

    // End point heading constraint
    A(num_splines * 4 - 1, num_splines * 4 - 4 + 1) = 1;
    A(num_splines * 4 - 1, num_splines * 4 - 4 + 2) = 2;
    A(num_splines * 4 - 1, num_splines * 4 - 4 + 3) = 3;

    return A;
}

std::vector<Eigen::Vector2d> getNormalVectors(const std::vector<Eigen::Vector2d>& path) {
    std::vector<Eigen::Vector2d> normal_vectors;

    if (path.size() < 2) {
        ROS_WARN("Cannot compute normal vectors: at least two points are required.");
        return normal_vectors;
    }

    normal_vectors.resize(path.size());

    for (size_t i = 0; i < path.size(); ++i) {
        Eigen::Vector2d normal;

        if (i == 0) {
            // Calculate normal at the start point
            double dx = path[i + 1].x() - path[i].x();
            double dy = path[i + 1].y() - path[i].y();
            normal = Eigen::Vector2d(-dy, dx);
        } else if (i == path.size() - 1) {
            // Calculate normal at the end point
            double dx = path[i].x() - path[i - 1].x();
            double dy = path[i].y() - path[i - 1].y();
            normal = Eigen::Vector2d(-dy, dx);
        } else {
            // Calculate normal for intermediate points
            double dx = path[i + 1].x() - path[i - 1].x();
            double dy = path[i + 1].y() - path[i - 1].y();
            normal = Eigen::Vector2d(-dy, dx);
        }

        normal.normalize();
        normal_vectors[i] = normal;
    }

    return normal_vectors;
}

// ========================================
// Boundaries
// ========================================
Boundaries getBoundaries(const std::vector<Eigen::Vector2d>& path, const std::vector<Eigen::Vector2d>& normal_vectors, bool local) {
    Boundaries boundaries;
    size_t n = path.size();
    boundaries.left_bnd.points.resize(n);
    boundaries.left_bnd.distances.resize(n);
    boundaries.right_bnd.points.resize(n);
    boundaries.right_bnd.distances.resize(n);

    for (size_t i = 0; i < n; ++i) {
        const auto& point = path[i];
        const auto& normal = normal_vectors[i];

        // The normal vector is defined in the left
        double left_dx = normal.x();
        double left_dy = normal.y();
        double right_dx = -left_dx;
        double right_dy = -left_dy;

        boundaries.left_bnd.distances[i] = checkBoundary(point, left_dx, left_dy, local);
        boundaries.right_bnd.distances[i] = checkBoundary(point, right_dx, right_dy, local);
    }

    constrainDeviationChange(boundaries.left_bnd.distances);
    constrainDeviationChange(boundaries.right_bnd.distances);

    for (size_t i = 0; i < n; ++i) {
        const auto& point = path[i];
        const auto& normal = normal_vectors[i];

        double left_dx = normal.x();
        double left_dy = normal.y();
        double right_dx = -left_dx;
        double right_dy = -left_dy;

        boundaries.left_bnd.points[i] = createBoundaryPoint(point, left_dx, left_dy, boundaries.left_bnd.distances[i]);
        boundaries.right_bnd.points[i] = createBoundaryPoint(point, right_dx, right_dy, boundaries.right_bnd.distances[i]);
    }

    return boundaries;
}

// version 1
// double checkBoundary(const Eigen::Vector2d& point, double dx, double dy) {
//     int x = point.x();
//     int y = point.y();
    
//     for (int d = 0; d < Constant::DEV_MAX; ++d) {
//         int nx = x + std::round(dx * d);
//         int ny = y + std::round(dy * d);
        
//         if (nx < 0 || ny < 0 || nx >= costmap->getSizeInCellsX() || ny >= costmap->getSizeInCellsY()) {
//             return d; // Return the distance at which boundary is reached
//         }

//         if (costmap->getCost(nx, ny) >= Constant::OBSTACLE_INFLATION) {
//             return d; // Return the distance at which an obstacle is encountered
//         }
//     }// Return the distance at which an obstacle is encountered or boundary is reached
//     return Constant::DEV_MAX; // Return the maximum distance if no obstacles are encountered
// }

// version 2
// double checkBoundary(const Eigen::Vector2d& point, double dx, double dy) {
//     int x = point.x();
//     int y = point.y();
    
//     int last_valid_d = Constant::DEV_MAX;

//     for (int d = Constant::DEV_MAX; d > 0; --d) {
//         int nx = x + std::round(dx * d);
//         int ny = y + std::round(dy * d);
        
//         if (nx < 0 || ny < 0 || nx >= costmap->getSizeInCellsX() || ny >= costmap->getSizeInCellsY()) {
//             // If we hit a boundary, return the last valid distance
//             return last_valid_d;
//         }

//         if (isClearLine(x, y, nx, ny)) {
//             // If we hit an obstacle, reduce the distance and continue
//             last_valid_d = d - 1;
//         } else {
//             // If no obstacle is found, return the last valid distance
//             return last_valid_d;
//         }
//     }

//     return last_valid_d; // Return the final valid distance if no obstacles are encountered within the loop
// }

// version 3
double checkBoundary(const Eigen::Vector2d& point, double dx, double dy, bool use_local_costmap) {
    double map_x = point.x();
    double map_y = point.y();
    // int map_x = static_cast<int>(point.x());
    // int map_y = static_cast<int>(point.y());

    int last_valid_d = 0;

    // Replanning
    if (use_local_costmap) {
        const nav_msgs::OccupancyGrid::ConstPtr& costmap_msg = local_costmap_msg;

        if (map_x < 0 || map_y < 0 || map_x >= costmap_msg->info.width || map_y >= costmap_msg->info.height) {
            // ROS_WARN("Point (%f, %f) is out of local costmap bounds (%d, %d).", map_x, map_y, costmap_msg->info.width, costmap_msg->info.height);
            return Constant::DEV_MAX;
        }

        bool found_valid_point = false;

        for (int d = 0; d < Constant::DEV_MAX; ++d) {
            int new_map_x = point.x() + d * dx;
            int new_map_y = point.y() + d * dy;

            if (new_map_x < 0 || new_map_y < 0 || new_map_x >= costmap_msg->info.width || new_map_y >= costmap_msg->info.height) {
                ROS_WARN("new_map_x or new_map_y is out of bounds.");
                break;
            }

            bool is_valid_point = true;
            int index = static_cast<int>(new_map_y * costmap_msg->info.width + new_map_x);
            // ROS_INFO("Checking index %d: costmap value = %d", index, costmap_msg->data[index]);

            unsigned char cost = costmap_msg->data[index];
            // std::cout << static_cast<int>(cost) << " ";
            if (cost >= Constant::LOCAL_OBSTACLE_INFLATION || cost == costmap_2d::NO_INFORMATION) {
                is_valid_point = false;
                ROS_INFO("Point is inside an obstacle or inflation zone.");
            }

            if (is_valid_point) {
                last_valid_d = d;
                found_valid_point = true;
                // ROS_INFO("Valid point found at distance %d", d);
            } else if (found_valid_point) {
                // ROS_INFO("Found a valid point before, stopping search.");
                break;
            }
        }

        // std::cout << '\n' << std::endl;

        // if (found_valid_point) {
        //     ROS_INFO("Last valid point found at distance %d", last_valid_d);
        // } else {
        //     ROS_WARN("No valid point found within %d iterations.", Constant::DEV_MAX);
        // }
    }
    // Normal planning
    else {
        if (!costmap) {
            ROS_WARN("Global costmap is not available.");
            return Constant::DEV_MAX;
        }

        unsigned int size_x = costmap->getSizeInCellsX();
        unsigned int size_y = costmap->getSizeInCellsY();

        if (map_x >= size_x || map_y >= size_y) {
            ROS_WARN("Point (%f, %f) is out of global costmap bounds.", map_x, map_y);
            return Constant::DEV_MAX;
        }

        bool found_valid_point = false;

        for (int d = 0; d < Constant::DEV_MAX; ++d) {
            int new_map_x = static_cast<int>(point.x() + d * dx);
            int new_map_y = static_cast<int>(point.y() + d * dy);

            if (new_map_x < 0 || new_map_y < 0 || new_map_x >= size_x || new_map_y >= size_y) {
                break;
            }

            bool is_valid_point = true;
            unsigned char cost = costmap->getCost(new_map_x, new_map_y);
            if (cost >= Constant::OBSTACLE_INFLATION || cost == costmap_2d::NO_INFORMATION) {
                is_valid_point = false;
            }

            if (is_valid_point) {
                last_valid_d = d;
                found_valid_point = true;
            } else if (found_valid_point) {
                break;
            }
        }
    }


    return last_valid_d;
}

void constrainDeviationChange(std::vector<double>& dev_list) {
    // Constrain forward direction
    double dev = dev_list[0];

    for (int i = 1; i < dev_list.size(); ++i) {
        double next_dev = dev + Constant::DEV_DOT;
        if (next_dev < dev_list[i]) {
            dev_list[i] = next_dev;
        }
        dev = dev_list[i];
    }

    // Constrain backward direction
    dev = dev_list.back();
    for (int i = dev_list.size() - 2; i >= 0; --i) {
        double prev_dev = dev + Constant::DEV_DOT;
        if (prev_dev < dev_list[i]) {
            dev_list[i] = prev_dev;
        }
        dev = dev_list[i];
    }
}

Eigen::Vector2d createBoundaryPoint(const Eigen::Vector2d& point, double dx, double dy, double distance) {
    return Eigen::Vector2d(point.x() + dx * distance, point.y() + dy * distance);
}

// ========================================
// Optimization
// ========================================
csc* denseToCSC(const Eigen::MatrixXd& mat) {
    int rows = mat.rows();
    int cols = mat.cols();
    
    // Step 1: Count the number of non-zero elements
    int nonzeros = 0;
    for (int j = 0; j < cols; ++j) {
        for (int i = 0; i < rows; ++i) {
            if (mat(i, j) != 0) {
                nonzeros++;
            }
        }
    }

    // Step 2: Allocate memory for CSC representation
    c_int* row_indices = (c_int*)c_malloc(nonzeros * sizeof(c_int));
    c_int* col_pointers = (c_int*)c_malloc((cols + 1) * sizeof(c_int));
    c_float* values = (c_float*)c_malloc(nonzeros * sizeof(c_float));

    // Step 3: Fill CSC representation
    int idx = 0;
    col_pointers[0] = 0;
    for (int j = 0; j < cols; ++j) {
        for (int i = 0; i < rows; ++i) {
            if (mat(i, j) != 0) {
                values[idx] = mat(i, j);
                row_indices[idx] = i;
                idx++;
            }
        }
        col_pointers[j + 1] = idx;
    }

    // Step 4: Return the CSC matrix
    return csc_matrix(rows, cols, nonzeros, values, row_indices, col_pointers);
}


double calculateSparsity(const Eigen::MatrixXd& mat) {
    int total_elements = mat.size();
    int non_zero_elements = (mat.array() != 0).count();
    double sparsity = 1.0 - static_cast<double>(non_zero_elements) / total_elements;
    ROS_INFO("Sparsity: %.2f%%", sparsity * 100);
    return sparsity;
}

std::pair<Eigen::VectorXd, int> getAlphaMinCurvature(const std::vector<Eigen::Vector2d>& path,
                                                     std::vector<double> left_dev_bnd,
                                                     std::vector<double> right_dev_bnd,
                                                     const std::vector<Eigen::Vector2d>& normal_vectors,
                                                     const Eigen::MatrixXd& A,
                                                     double curvature_bnd,
                                                     double start_angle, double end_angle, double gamma) {
    std::size_t num_points = path.size();
    std::size_t num_splines = num_points - 1;

    // Construct extraction matrix : b component extraction for first derivative x'(t=0)
    Eigen::MatrixXd A_ex_b = Eigen::MatrixXd::Zero(num_points, num_splines * 4);
    for (int i = 0; i < num_splines; ++i) {
        A_ex_b(i, i * 4 + 1) = 1;
    }
    A_ex_b.row(num_points - 1).tail(4) << 0, 1, 2, 3; // extract first derivative x'(t=1) in the last point

    // Construct extraction matrix : c component extraction for second derivative x''(t=0)
    Eigen::MatrixXd A_ex_c = Eigen::MatrixXd::Zero(num_points, num_splines * 4);
    for (int i = 0; i < num_splines; ++i) {
        A_ex_c(i, i * 4 + 2) = 2;
    }
    A_ex_c.row(num_points - 1).tail(4) << 0, 0, 2, 6; // extract second derviative x''(t=1) in the last point

    // Construct extraction matirx : b, c, d components extraction
    Eigen::MatrixXd A_ex_bcd = Eigen::MatrixXd::Zero(num_points * 3, num_splines * 4);
    for (int i = 0; i < num_splines; ++i) {
        A_ex_bcd(i * 3    , i * 4 + 1) = 1;
        A_ex_bcd(i * 3 + 1, i * 4 + 2) = 1;
        A_ex_bcd(i * 3 + 2, i * 4 + 3) = 1;
    }

    // Eigen::MatrixXd A_inv = A.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXd A_inv = A.inverse();
    Eigen::MatrixXd T_c = A_ex_c * A_inv;

    // Construct normal vector matrix
    Eigen::MatrixXd N_x = Eigen::MatrixXd::Zero(num_splines * 4, num_points);
    Eigen::MatrixXd N_y = Eigen::MatrixXd::Zero(num_splines * 4, num_points);

    for (int i = 0; i < num_splines; ++i) {
        int j = i * 4;
        const auto& normal = normal_vectors[i];

        if (i < num_points - 1) {
            const auto& normal_next = normal_vectors[i + 1];

            N_x(j, i) = normal.x();
            N_x(j + 1, i + 1) = normal_next.x();
            N_y(j, i) = normal.y();
            N_y(j + 1, i + 1) = normal_next.y();
        } else {
            N_x(j, i) = normal.x();
            N_x(j + 1, 0) = normal_vectors[0].x();

            N_y(j, i) = normal.y();
            N_y(j + 1, 0) = normal_vectors[0].y();
        }
    }

    // Construct reference path vector
    Eigen::VectorXd p_x = Eigen::VectorXd::Zero(num_splines * 4);
    Eigen::VectorXd p_y = Eigen::VectorXd::Zero(num_splines * 4);

    for (int i = 0; i < num_splines; ++i) {
        int j = i * 4;
        const auto& point = path[i];

        if (i < num_points - 1) {
            const auto& point_next = path[i + 1];

            p_x(j) = point.x();
            p_x(j + 1) = point_next.x();

            p_y(j) = point.y();
            p_y(j + 1) = point_next.y();
        } else {
            p_x(j) = point.x();
            p_x(j + 1) = path[0].x();

            p_y(j) = point.y();
            p_y(j + 1) = path[0].y();
        }
    }

    p_x(num_splines * 4 - 2) = std::cos(start_angle);
    p_y(num_splines * 4 - 2) = std::sin(start_angle);
    p_x(num_splines * 4 - 1) = std::cos(end_angle);
    p_y(num_splines * 4 - 1) = std::sin(end_angle);

    // Calculate derivatives: x', y', x'y'
    Eigen::VectorXd x_dot = A_ex_b * A_inv * p_x;
    Eigen::VectorXd y_dot = A_ex_b * A_inv * p_y;
    Eigen::VectorXd x_dot_y_dot = -2 * x_dot.array() * y_dot.array();

    // Square of derivatives
    Eigen::VectorXd x_dot_sq = x_dot.array().square();
    Eigen::VectorXd y_dot_sq = y_dot.array().square();

    // Curvature denominator calculations
    Eigen::VectorXd curv_den = (x_dot_sq + y_dot_sq).array().pow(1.5);
    curv_den = curv_den.unaryExpr([](double v) { return std::abs(v) < 1e-9 ? 1e-9 : v; });
    Eigen::VectorXd curvatures_den = curv_den.array().inverse();
    curvatures_den = curvatures_den.unaryExpr([](double v) { return std::isfinite(v) ? v : 0.0; });
    Eigen::VectorXd curvatures_sq_den = curvatures_den.array().square();

    // Energy penalty
    Eigen::MatrixXd T_q = A_ex_bcd * A_inv;
    Eigen::MatrixXd T_qnx = T_q * N_x;
    Eigen::MatrixXd T_qny = T_q * N_y;

    Eigen::MatrixXd Q_E = Eigen::MatrixXd::Zero(num_points * 3, num_points * 3);
    Eigen::MatrixXd block(3, 3);
    block << 1, 1, 1,
             1, 4.0/3.0, 3.0/2.0,
             1, 3.0/2.0, 9.0/5.0;

    for (int i = 0; i < num_points; ++i) {
        Q_E.block<3,3>(i * 3, i * 3) = block;
    }
    
    // Objective function formulation
    Eigen::MatrixXd P_xx = (curvatures_sq_den.array() * y_dot_sq.array()).matrix().asDiagonal();
    Eigen::MatrixXd P_yy = (curvatures_sq_den.array() * x_dot_sq.array()).matrix().asDiagonal();
    Eigen::MatrixXd P_xy = (curvatures_sq_den.array() * x_dot_y_dot.array()).matrix().asDiagonal();

    Eigen::MatrixXd T_nx = T_c * N_x;
    Eigen::MatrixXd T_ny = T_c * N_y;

    Eigen::MatrixXd H_x = T_nx.transpose() * P_xx * T_nx;
    Eigen::MatrixXd H_xy = T_ny.transpose() * P_xy * T_nx;
    Eigen::MatrixXd H_y = T_ny.transpose() * P_yy * T_ny;
    Eigen::MatrixXd H_E = T_qnx.transpose() * Q_E * T_qnx + T_qny.transpose() * Q_E * T_qny;
    Eigen::MatrixXd H_k = H_x + H_xy + H_y;
    Eigen::MatrixXd H = H_k + gamma * H_E;
    H = (H + H.transpose()) * 0.5;
    // double regularization = 1e-6;
    // H += regularization * Eigen::MatrixXd::Identity(H.rows(), H.cols());

    Eigen::VectorXd f_x = 2 * p_x.transpose() * T_c.transpose() * P_xx * T_nx;
    Eigen::VectorXd f_y = 2 * p_y.transpose() * T_c.transpose() * P_yy * T_ny;
    Eigen::VectorXd f_xy = p_x.transpose() * T_c.transpose() * P_xy * T_ny + p_y.transpose() * T_c.transpose() * P_xy * T_nx;
    Eigen::VectorXd f_k = f_x + f_xy + f_y;
    Eigen::VectorXd f_E = 2 * T_qnx.transpose() * Q_E * T_q * p_x + 2 * T_qny.transpose() * Q_E * T_q * p_y;
    Eigen::VectorXd f = f_k + gamma * f_E;

    // Constraint function formulation
    Eigen::VectorXd P_x = curvatures_den.array() * y_dot.array();
    Eigen::VectorXd P_y = curvatures_den.array() * x_dot.array();

    Eigen::MatrixXd E_curvature = P_y.asDiagonal() * T_ny - P_x.asDiagonal() * T_nx;
    Eigen::VectorXd curvature_reference = P_y.asDiagonal() * T_c * p_y - P_x.asDiagonal() * T_c * p_x;

    Eigen::VectorXd curv_constraints_ge = Eigen::VectorXd::Constant(num_points, curvature_bnd) - curvature_reference;
    Eigen::VectorXd curv_constraints_le = -Eigen::VectorXd::Constant(num_points, curvature_bnd) - curvature_reference;

    // Boundary constraints
    double end_point_boundary = 0.05;
    left_dev_bnd[0] = end_point_boundary;
    right_dev_bnd[0] = end_point_boundary;
    left_dev_bnd[num_points - 1] = end_point_boundary;
    right_dev_bnd[num_points - 1] = end_point_boundary;

    // Checking conditions
    for (size_t i = 0; i < num_points; ++i) {
        if (-right_dev_bnd[i] > left_dev_bnd[i] || -left_dev_bnd[i] > right_dev_bnd[i]) {
            throw std::runtime_error("ERROR: Too small track");
        }
    }

    // Matrix stacking and appending: E, u, l
    Eigen::MatrixXd E(2 * num_points, num_points);
    E.topRows(num_points) = Eigen::MatrixXd::Identity(num_points, num_points);
    E.bottomRows(num_points) = E_curvature;

    Eigen::VectorXd u(2 * num_points);
    Eigen::VectorXd l(2 * num_points);
    for (size_t i = 0; i < num_points; ++i) {
        u(i) = left_dev_bnd[i];
        l(i) = -right_dev_bnd[i];
    }
    u.segment(num_points, num_points) = curv_constraints_ge;
    l.segment(num_points, num_points) = curv_constraints_le;

    // // Convert Eigen matrices/vectors to string for ROS_INFO
    // std::stringstream ss_E, ss_u, ss_l;
    // // ss_E << E;
    // ss_u << (E_curvature.inverse() * curv_constraints_ge).transpose();
    // ss_l << (E_curvature.inverse() * curv_constraints_le).transpose();

    // // ROS_INFO_STREAM to output the contents
    // // ROS_INFO_STREAM("Matrix E:\n" << ss_E.str());
    // ROS_INFO_STREAM("Vector u:\n" << ss_u.str());
    // ROS_INFO_STREAM("Vector l:\n" << ss_l.str());

    // // Check convexity by inspecting the eigenvalues of H
    // Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(H);
    // ROS_INFO_STREAM("Eigenvalues of H: " << eigen_solver.eigenvalues().transpose());

    // Sparsity calculation and Sparse/Dense optimization branching
    // double sparsity_threshold = 0.7;  // Consider as sparse if more than 90% are zeros
    // double H_sparsity = calculateSparsity(H);
    // double E_sparsity = calculateSparsity(E);

    // Use upper triangular part of H
    Eigen::MatrixXd H_upper = H.triangularView<Eigen::Upper>();

    // Use OSQP settings as usual for dense matrix optimization
    OSQPSettings *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    osqp_set_default_settings(settings);
    settings->rho = 0.1;
    settings->alpha = 1.6;
    settings->verbose = 0;
    settings->warm_start = 1;

    // OSQP data
    OSQPData *data = (OSQPData *)c_malloc(sizeof(OSQPData));
    data->n = H_upper.rows();
    data->m = E.rows();
    data->P = denseToCSC(H_upper);
    data->q = f.data();
    data->A = denseToCSC(E);
    data->l = l.data();
    data->u = u.data();

    // Setup workspace
    OSQPWorkspace *work;
    osqp_setup(&work, data, settings);

    // Solve the problem
    osqp_solve(work);
    Eigen::VectorXd solution(data->n);
    for (int i = 0; i < data->n; ++i) {
        solution[i] = work->solution->x[i];
    }

    int success = work->info->status_val;

    // Clean up
    osqp_cleanup(work);
    c_free(data->P->x);
    c_free(data->P->i);
    c_free(data->P->p);
    c_free(data->P);
    c_free(data->A->x);
    c_free(data->A->i);
    c_free(data->A->p);
    c_free(data->A);
    c_free(data);
    c_free(settings);

    return std::make_pair(solution, success);
}

std::vector<Eigen::Vector2d> getPathMinCurvature(const std::vector<Eigen::Vector2d>& path,
                                                 const std::vector<Eigen::Vector2d>& normal_vectors,
                                                 const Eigen::VectorXd& alpha) {
    std::vector<Eigen::Vector2d> new_path(path.size());

    for (size_t i = 0; i < path.size(); ++i) {
        new_path[i] = path[i] + alpha(i) * normal_vectors[i];
        
        // std::ostringstream oss;
        // oss << "Iteration " << i << ":\n"
        //     << "  path[" << i << "]: (" << path[i].x() << ", " << path[i].y() << ")\n"
        //     << "  normal_vectors[" << i << "]: (" << normal_vectors[i].x() << ", " << normal_vectors[i].y() << ")\n"
        //     << "  alpha[" << i << "]: " << alpha(i) << "\n"
        //     << "  new_path[" << i << "]: (" << new_path[i].x() << ", " << new_path[i].y() << ")";

        // ROS_INFO_STREAM(oss.str());
    }

    return new_path;
}

// ========================================
// Utilities
// ========================================
Eigen::VectorXd getCurvatureUsingSpline(const std::vector<Eigen::Vector2d>& path, const Eigen::MatrixXd& A, double start_angle, double end_angle) {
    int num_points = path.size();
    int num_splines = num_points - 1;

    if (std::isnan(start_angle)) {
        Eigen::Vector2d firstSegment = path[1] - path[0];
        start_angle = std::atan2(firstSegment.y(), firstSegment.x());
    }
    if (std::isnan(end_angle)) {
        Eigen::Vector2d lastSegment = path[num_points - 1] - path[num_points - 2];
        end_angle = std::atan2(lastSegment.y(), lastSegment.x());
    }

    // Construct b component extraction matrix
    Eigen::MatrixXd A_ex_b = Eigen::MatrixXd::Zero(num_points, num_splines * 4);
    for (int i = 0; i < num_splines; ++i) {
        A_ex_b(i, i * 4 + 1) = 1;
    }
    A_ex_b.bottomRightCorner(1, 4) << 0, 1, 2, 3;

    // Construct c component extraction matrix
    Eigen::MatrixXd A_ex_c = Eigen::MatrixXd::Zero(num_points, num_splines * 4);
    for (int i = 0; i < num_splines; ++i) {
        A_ex_c(i, i * 4 + 2) = 2;
    }
    A_ex_c.bottomRightCorner(1, 4) << 0, 0, 2, 6;

    Eigen::MatrixXd A_inv = A.inverse();
    Eigen::MatrixXd T_c = A_ex_c * A_inv;

    // Construct reference path matrix
    Eigen::VectorXd p_x = Eigen::VectorXd::Zero(num_splines * 4);
    Eigen::VectorXd p_y = Eigen::VectorXd::Zero(num_splines * 4);
    for (int i = 0; i < num_splines; ++i) {
        int j = i * 4;
        p_x.segment(j, 2) << path[i].x(), path[i + 1].x();
        p_y.segment(j, 2) << path[i].y(), path[i + 1].y();
    }
    p_x(num_splines * 4 - 2) = std::cos(start_angle);
    p_y(num_splines * 4 - 2) = std::sin(start_angle);
    p_x(num_splines * 4 - 1) = std::cos(end_angle);
    p_y(num_splines * 4 - 1) = std::sin(end_angle);

    // Calculate derivatives
    Eigen::VectorXd x_dot = A_ex_b * A_inv * p_x;
    Eigen::VectorXd y_dot = A_ex_b * A_inv * p_y;

    Eigen::VectorXd x_dot_sq = x_dot.array().square();
    Eigen::VectorXd y_dot_sq = y_dot.array().square();

    // Calculate curvature denominator
    Eigen::VectorXd curv_den = (x_dot_sq + y_dot_sq).array().pow(1.5);
    Eigen::VectorXd curvatures_den = (1.0 / curv_den.array()).matrix();

    Eigen::VectorXd P_x = curvatures_den.asDiagonal() * y_dot;
    Eigen::VectorXd P_y = curvatures_den.asDiagonal() * x_dot;

    Eigen::VectorXd curvature = P_y.asDiagonal() * T_c * p_y - P_x.asDiagonal() * T_c * p_x;

    return curvature;
}

double getTravelTime(const std::vector<Eigen::Vector2d>& path, const Eigen::VectorXd& velocity_profile) {
    std::vector<Eigen::Vector2d> deltas;
    for (size_t i = 1; i < path.size(); ++i) {
        deltas.push_back(path[i] - path[i-1]);
    }

    std::vector<double> distances(deltas.size());
    for (size_t i = 0; i < deltas.size(); ++i) {
        distances[i] = deltas[i].norm();
    }

    double travel_length = std::accumulate(distances.begin(), distances.end(), 0.0);
    double avg_vel = velocity_profile.mean();
    double travel_time = travel_length / avg_vel;

    return travel_time;
}

} // namespace my_global_planner

#pragma GCC pop_options
