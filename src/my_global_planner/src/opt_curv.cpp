// opt_curv.cpp
#include <ros/ros.h>
#include <ros/param.h>
#include <Eigen/Sparse>
#include <cmath>
#include <iostream>
#include <limits>
#include <algorithm>
#include <tf/tf.h>
#include <numeric>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
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
ros::Publisher travel_time_publisher;
ros::Publisher curvature_sums_publisher;
ros::Publisher curvature_peaks_publisher;
ros::Publisher index_publisher;

// Use EPSILON to treat very small changes in travel_time as no change,
double EPSILON = 1e-6;

// ========================================
// Main
// ========================================
std::vector<Eigen::Vector2d> optimize_min_curvature(std::vector<Eigen::Vector2d> path, const size_t max_iter, bool local, tf2_ros::Buffer& tf_buffer) {
    
    std::vector<Eigen::Vector2d> new_path = path;
    std::vector<double> travel_times;
    std::vector<double> curvature_sums;
    std::vector<double> curvature_peaks;
    Boundaries current_boundaries;
    Boundaries prev_boundaries;
    bool success_flag=true;
    int i;

    int qp_type;
    ros::NodeHandle nh("/");
    nh.param("move_base/QP_type", qp_type, 1);
    ROS_INFO("============ QP optimizatoin Start ============");
    if (qp_type == 0) {
        ROS_INFO("Use uniformSpacing");
    } else if (qp_type == 1) {
        ROS_INFO("Use adaptiveSpacing");
    } else {
        ROS_ERROR("Invalid QP_type parameter: %d. Expected values: 0 (uniformSpacing), 1 (adaptiveSpacing)", qp_type);
        return new_path;
    }

    ros::WallTime wall_qp_start = ros::WallTime::now();
    double wall_time_spacing = 0.0;
    double wall_time_matrix_normals = 0.0;
    double wall_time_boundaries = 0.0;
    double wall_time_transform = 0.0;
    double wall_time_publish = 0.0;
    double wall_time_osqp = 0.0;
    double wall_time_curvature = 0.0;
    double wall_time_travel = 0.0;

    auto update_stage = [](ros::WallTime& start, double& slot) {
        ros::WallTime now = ros::WallTime::now();
        slot += (now - start).toSec();
        start = now;
    };

    for (i = 0; i < max_iter; ++i) {
        ros::WallTime stage_start = ros::WallTime::now();

        if (qp_type == 0) {
            path = uniformSpacing(new_path, Constant::UNIFORM_STEP);
        } else if (qp_type == 1) {
            path = adaptiveSpacing(new_path, Constant::ADAPTIVE_MIN_STEP, Constant::ADAPTIVE_MAX_STEP);
        }
        update_stage(stage_start, wall_time_spacing);

        Eigen::MatrixXd A = getSystemMatrix(path);
        std::vector<Eigen::Vector2d> normal_vectors = getNormalVectors(path);
        update_stage(stage_start, wall_time_matrix_normals);

        // Boundaries
        if (i != 0) {
            prev_boundaries = current_boundaries;
        }
        current_boundaries = getBoundaries(path, normal_vectors, local);
        update_stage(stage_start, wall_time_boundaries);

        // Publish Boundary Lines
        // Normal planning
        if (!local){
            std::vector<geometry_msgs::PoseStamped> left_bnd = convertVector2dToPoseStamped(current_boundaries.left_bnd.points);
            std::vector<geometry_msgs::PoseStamped> right_bnd = convertVector2dToPoseStamped(current_boundaries.right_bnd.points);
            nav_msgs::Path left_bnd_world = producePathMapToWorld(left_bnd);
            nav_msgs::Path right_bnd_world = producePathMapToWorld(right_bnd);
            publishPath(plan_publisher_bnd_L, left_bnd_world);
            publishPath(plan_publisher_bnd_R, right_bnd_world);
            update_stage(stage_start, wall_time_publish);
        }
        // Replanning
        else {
            std::vector<Eigen::Vector2d> left = current_boundaries.left_bnd.points;
            std::vector<Eigen::Vector2d> right = current_boundaries.right_bnd.points;

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

            update_stage(stage_start, wall_time_transform);

            // Convert the points to PoseStamped and publish
            std::vector<geometry_msgs::PoseStamped> left_bnd_plan = convertVector2dToPoseStamped(left);
            std::vector<geometry_msgs::PoseStamped> right_bnd_plan = convertVector2dToPoseStamped(right);
            nav_msgs::Path left_bnd_replan_world = producePathMapToWorld(left_bnd_plan);
            nav_msgs::Path right_bnd_replan_world = producePathMapToWorld(right_bnd_plan);
            publishPath(plan_publisher_bnd_L_replan, left_bnd_replan_world);
            publishPath(plan_publisher_bnd_R_replan, right_bnd_replan_world);
            update_stage(stage_start, wall_time_publish);
        }

        // Calculate start and end angles
        Eigen::Vector2d start_segment = path[1] - path[0];
        Eigen::Vector2d end_segment = path[path.size() - 1] - path[path.size() - 2];
        double start_angle = std::atan2(start_segment.y(), start_segment.x());
        double end_angle = std::atan2(end_segment.y(), end_segment.x());

        // Select curvature bound method (constant/ exponential decay)
        // double curvature_bound = CURVATURE_BOUND;
        double curvature_bound = (Constant::CURVATURE_BOUND_MAX - Constant::CURVATURE_BOUND_MIN) * std::exp(-i * Constant::CURVATURE_BOUND_DECAY) + Constant::CURVATURE_BOUND_MIN;

        // Alpha calculation
        ros::WallTime osqp_stage_start = ros::WallTime::now();
        auto result = getAlphaMinCurvature(path, current_boundaries.left_bnd.distances, current_boundaries.right_bnd.distances, normal_vectors, A, curvature_bound, start_angle, end_angle, Constant::GAMMA);
        Eigen::VectorXd alpha = result.first;
        int success = result.second;
        wall_time_osqp += (ros::WallTime::now() - osqp_stage_start).toSec();

        if (success == OSQP_SOLVED) {
            new_path = getPathMinCurvature(path, normal_vectors, alpha);
            std::vector<geometry_msgs::PoseStamped> new_path_tmp = convertVector2dToPoseStamped(new_path);
            nav_msgs::Path new_path_world = producePathMapToWorld(new_path_tmp);
            publishPath(plan_publisher_opt, new_path_world);
        } else {
            ROS_INFO("Optimization Failed at step [%d]", i + 1);
            new_path = path;
            success_flag = false;
            // Reverting to the previous boundaries
            std::vector<geometry_msgs::PoseStamped> left_bnd_prev = convertVector2dToPoseStamped(prev_boundaries.left_bnd.points);
            std::vector<geometry_msgs::PoseStamped> right_bnd_prev = convertVector2dToPoseStamped(prev_boundaries.right_bnd.points);
            nav_msgs::Path left_bnd_prev_world = producePathMapToWorld(left_bnd_prev);
            nav_msgs::Path right_bnd_prev_world = producePathMapToWorld(right_bnd_prev);
            publishPath(plan_publisher_bnd_L, left_bnd_prev_world);
            publishPath(plan_publisher_bnd_R, right_bnd_prev_world);
            break;
        }

        // Convert the path from px to meter for curvature calculation, velocity profile generation and travel time calculation
        std::vector<Eigen::Vector2d> new_path_m = convertPx2Meter(new_path);

        // Get curvature and velocity profile
        ros::WallTime curvature_stage_start = ros::WallTime::now();
        double path_length = getPathLength(new_path_m);
        double path_energy = getPathEnergy(new_path_m);
        ROS_INFO("Opt. Step [%d] | Path Length: %f", i + 1, path_length);
        ROS_INFO("Opt. Step [%d] | Path Energy: %f", i + 1, path_energy);
        Eigen::VectorXd curvature = getCurvatureUsingFiniteDifference(new_path_m);
        auto velocity_result = getVelocityProfile(new_path_m, curvature, Constant::INIT_VELOCITY, Constant::MAX_VEL, Constant::MAX_AX, Constant::MAX_AY);
        wall_time_curvature += (ros::WallTime::now() - curvature_stage_start).toSec();
        Eigen::VectorXd velocity_profile = velocity_result.first;

        double curvature_squared_sum = curvature.squaredNorm();
        ROS_INFO("Opt. Step [%d] | Sum of Squared Curvature: %f", i + 1, curvature_squared_sum);
        curvature_sums.push_back(curvature_squared_sum);

        double max_curvature = curvature.cwiseAbs().maxCoeff();
        ROS_INFO("Opt. Step [%d] | Max Curvature: %f", i + 1, max_curvature);
        curvature_peaks.push_back(max_curvature);
        
        // Get travel time
        ros::WallTime travel_stage_start = ros::WallTime::now();
        double travel_time = getTravelTime(new_path_m, velocity_profile);
        wall_time_travel += (ros::WallTime::now() - travel_stage_start).toSec();
        ROS_INFO("Opt. Step [%d] | Expected Travel Time: %f sec", i + 1, travel_time);
        travel_times.push_back(travel_time);

        // Compare current travel time with previous iteration
        // if (i >= 3 && (travel_time > travel_times[i - 1] || std::isnan(travel_time))) {
        //     ROS_INFO("Travel time increased. Reverting to path at step [%d]", i);
        //     // Revert to the previous path
        //     new_path = path; 
        //     // Revert to the previous boundaries
        //     std::vector<geometry_msgs::PoseStamped> left_bnd_prev = convertVector2dToPoseStamped(prev_boundaries.left_bnd.points);
        //     std::vector<geometry_msgs::PoseStamped> right_bnd_prev = convertVector2dToPoseStamped(prev_boundaries.right_bnd.points);
        //     nav_msgs::Path left_bnd_prev_world = producePathMapToWorld(left_bnd_prev);
        //     nav_msgs::Path right_bnd_prev_world = producePathMapToWorld(right_bnd_prev);
        //     publishPath(plan_publisher_bnd_L, left_bnd_prev_world);
        //     publishPath(plan_publisher_bnd_R, right_bnd_prev_world);

        //     success_flag = true;
        //     break;
        // }
    }

    ros::WallTime wall_qp_end = ros::WallTime::now();
    double wall_time_total_qp = (wall_qp_end - wall_qp_start).toSec();
    double accounted = wall_time_spacing + wall_time_matrix_normals + wall_time_boundaries +
                       wall_time_transform + wall_time_publish + wall_time_osqp +
                       wall_time_curvature + wall_time_travel;
    double wall_time_other = wall_time_total_qp - accounted;
    // ROS_INFO_STREAM("[Timing][QP] total=" << wall_time_total_qp
    //                 << " | spacing=" << wall_time_spacing
    //                 << ", matrix_normals=" << wall_time_matrix_normals
    //                 << ", boundaries=" << wall_time_boundaries
    //                 << ", transform=" << wall_time_transform
    //                 << ", publish=" << wall_time_publish
    //                 << ", osqp=" << wall_time_osqp
    //                 << ", curvature=" << wall_time_curvature
    //                 << ", travel=" << wall_time_travel
    //                 << ", other=" << wall_time_other);

    std_msgs::Bool status_msg;
    status_msg.data = success_flag;
    optimization_status_publisher.publish(status_msg);

    std_msgs::Int32 index_msg;
    index_msg.data = i - 1;
    index_publisher.publish(index_msg);

    publishTravelTimes(travel_time_publisher, travel_times);
    publishCurvatureSums(curvature_sums_publisher, curvature_sums);
    publishCurvaturePeaks(curvature_peaks_publisher, curvature_peaks);

    ROS_INFO("============= QP optimization End =============");
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

    // Create the boundary tube (lateral distance)
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

    // Create right normal vectors
    std::vector<Eigen::Vector2d> inverted_normal_vectors(normal_vectors.size());
    for (size_t i = 0; i < normal_vectors.size(); ++i) {
        inverted_normal_vectors[i] = -normal_vectors[i];
    }

    // Constrain the deviation to prevent boundary crossing
    constrainDeviationToPreventCrossing(path, normal_vectors, boundaries.left_bnd.distances);
    constrainDeviationToPreventCrossing(path, inverted_normal_vectors, boundaries.right_bnd.distances);

    // Constrain the deviation change
    // constrainDeviationChange(boundaries.left_bnd.distances);
    // constrainDeviationChange(boundaries.right_bnd.distances);

    // Create boundary points to publish
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

void constrainDeviationToPreventCrossing(const std::vector<Eigen::Vector2d>& path, const std::vector<Eigen::Vector2d>& normal_vectors, std::vector<double>& dev_list) {
    for (size_t i = 0; i < path.size() - 1; ++i) {
        double n1x = normal_vectors[i].x();
        double n1y = normal_vectors[i].y();
        double n2x = normal_vectors[i + 1].x();
        double n2y = normal_vectors[i + 1].y();

        double p1x = path[i].x();
        double p1y = path[i].y();
        double p2x = path[i + 1].x();
        double p2y = path[i + 1].y();

        double determinant = -n1x * n2y + n1y * n2x;

        // Parallel lines
        if (std::abs(determinant) == 0.0) {
            continue;
        }

        double t = std::abs((-n2y * (p2x - p1x) + n2x * (p2y - p1y)) / determinant);
        double s = std::abs((n1x * (p2y - p1y) - n1y * (p2x - p1x)) / determinant);

        if (t < dev_list[i]) {
            dev_list[i] = std::max(0.0, t - Constant::DEV_EPSILON);
            // ROS_INFO("Cutting deviation at index %zu, t = %f", i, t);
        }
        if (s < dev_list[i + 1]) {
            dev_list[i + 1] = std::max(0.0, s - Constant::DEV_EPSILON);
            // ROS_INFO("Cutting deviation at index %zu, s = %f", i + 1, s);
        }
    }
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

void logMatrixSparsity(const std::string& name, const Eigen::MatrixXd& mat) {
    int total = mat.size();
    int nnz = (mat.array() != 0).count();
    double sparsity = 1.0 - static_cast<double>(nnz) / std::max(1, total);
    ROS_INFO_STREAM("[Sparsity] " << name << " nnz=" << nnz << "/" << total
                    << " (" << sparsity * 100.0 << "% zero)");
}

void logMatrixSparsity(const std::string& name, const Eigen::SparseMatrix<double>& mat) {
    int total = mat.rows() * mat.cols();
    int nnz = mat.nonZeros();
    double sparsity = 1.0 - static_cast<double>(nnz) / std::max(1, total);
    ROS_INFO_STREAM("[Sparsity] " << name << " nnz=" << nnz << "/" << total
                    << " (" << sparsity * 100.0 << "% zero)");
}

std::pair<Eigen::VectorXd, int> getAlphaMinCurvature(const std::vector<Eigen::Vector2d>& path,
                                                     std::vector<double> left_dev_bnd,
                                                     std::vector<double> right_dev_bnd,
                                                     const std::vector<Eigen::Vector2d>& normal_vectors,
                                                     const Eigen::MatrixXd& A,
                                                     double curvature_bnd,
                                                     double start_angle, double end_angle, double gamma) {

    ros::WallTime wall_getalpha_start = ros::WallTime::now();
    ros::WallTime wall_stage_start = wall_getalpha_start;
    
    auto accum_stage = [&](double& slot) {
        ros::WallTime now = ros::WallTime::now();
        slot += (now - wall_stage_start).toSec();
        wall_stage_start = now;
    };

    double wall_time_matrix_build = 0.0;
    double wall_time_csc = 0.0;
    double wall_time_setup = 0.0;
    double wall_time_solve = 0.0;
    double wall_time_cleanup = 0.0;

    // Fine-grained matrix-build timing
    ros::WallTime wall_build = ros::WallTime::now();
    double t_triplet = 0.0;
    double t_factor = 0.0;
    double t_Tc = 0.0;
    double t_normal_triplet = 0.0;
    double t_normal_set = 0.0;
    double t_solve_px = 0.0;
    double t_solve_py = 0.0;
    double t_curv = 0.0;
    double t_energy = 0.0;
    double t_H = 0.0;
    double t_constraints = 0.0;
    auto tick = [&](double& slot) {
        ros::WallTime now = ros::WallTime::now();
        slot += (now - wall_build).toSec();
        wall_build = now;
    };

    std::size_t num_points = path.size();
    std::size_t num_splines = num_points - 1;

    // Construct Sparse extraction matrix : b component extraction for first derivative x'(t=0)
    typedef Eigen::Triplet<double> T;
    std::vector<T> triplets_b;
    std::vector<T> triplets_c;
    std::vector<T> triplets_bcd;
    triplets_b.reserve(num_splines + 3);
    triplets_c.reserve(num_splines + 2);
    triplets_bcd.reserve(num_splines * 3);

    for (int i = 0; i < num_splines; ++i) {
        triplets_b.emplace_back(i, i * 4 + 1, 1.0);
        triplets_c.emplace_back(i, i * 4 + 2, 2.0);

        triplets_bcd.emplace_back(i * 3    , i * 4 + 1, 1.0);
        triplets_bcd.emplace_back(i * 3 + 1, i * 4 + 2, 1.0);
        triplets_bcd.emplace_back(i * 3 + 2, i * 4 + 3, 1.0);
    }
    int last_base = static_cast<int>(num_splines * 4 - 4);
    triplets_b.emplace_back(num_points - 1, last_base + 1, 1.0);
    triplets_b.emplace_back(num_points - 1, last_base + 2, 2.0);
    triplets_b.emplace_back(num_points - 1, last_base + 3, 3.0);

    triplets_c.emplace_back(num_points - 1, last_base + 2, 2.0);
    triplets_c.emplace_back(num_points - 1, last_base + 3, 6.0);

    Eigen::SparseMatrix<double> A_ex_b(num_points, num_splines * 4);
    Eigen::SparseMatrix<double> A_ex_c(num_points, num_splines * 4);
    Eigen::SparseMatrix<double> A_ex_bcd(num_points * 3, num_splines * 4);
    A_ex_b.setFromTriplets(triplets_b.begin(), triplets_b.end());
    A_ex_c.setFromTriplets(triplets_c.begin(), triplets_c.end());
    A_ex_bcd.setFromTriplets(triplets_bcd.begin(), triplets_bcd.end());
    tick(t_triplet);

    Eigen::SparseMatrix<double> A_sp = A.sparseView();
    Eigen::SparseMatrix<double> AT_sp = A_sp.transpose();
    Eigen::SparseLU<Eigen::SparseMatrix<double>> A_solver;
    Eigen::SparseLU<Eigen::SparseMatrix<double>> AT_solver;
    A_solver.analyzePattern(A_sp);
    A_solver.factorize(A_sp);
    AT_solver.analyzePattern(AT_sp);
    AT_solver.factorize(AT_sp);
    tick(t_factor);
    Eigen::MatrixXd T_c = AT_solver.solve(Eigen::MatrixXd(A_ex_c.transpose())).transpose();
    tick(t_Tc);

    // Construct normal vector matrix (sparse)
    std::vector<T> triplets_Nx;
    std::vector<T> triplets_Ny;
    triplets_Nx.reserve(num_splines * 2);
    triplets_Ny.reserve(num_splines * 2);

    for (int i = 0; i < num_splines; ++i) {
        int j = i * 4;
        const auto& normal = normal_vectors[i];

        if (i < num_points - 1) {
            const auto& normal_next = normal_vectors[i + 1];

            triplets_Nx.emplace_back(j, i, normal.x());
            triplets_Nx.emplace_back(j + 1, i + 1, normal_next.x());
            triplets_Ny.emplace_back(j, i, normal.y());
            triplets_Ny.emplace_back(j + 1, i + 1, normal_next.y());
        } else {
            triplets_Nx.emplace_back(j, i, normal.x());
            triplets_Nx.emplace_back(j + 1, 0, normal_vectors[0].x());

            triplets_Ny.emplace_back(j, i, normal.y());
            triplets_Ny.emplace_back(j + 1, 0, normal_vectors[0].y());
        }
    }
    tick(t_normal_triplet);

    Eigen::SparseMatrix<double> N_x(num_splines * 4, num_points);
    Eigen::SparseMatrix<double> N_y(num_splines * 4, num_points);
    N_x.setFromTriplets(triplets_Nx.begin(), triplets_Nx.end());
    N_y.setFromTriplets(triplets_Ny.begin(), triplets_Ny.end());
    tick(t_normal_set);

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
    Eigen::VectorXd px_solution = A_solver.solve(p_x);
    tick(t_solve_px);
    Eigen::VectorXd py_solution = A_solver.solve(p_y);
    tick(t_solve_py);
    Eigen::VectorXd x_dot = A_ex_b * px_solution;
    Eigen::VectorXd y_dot = A_ex_b * py_solution;
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
    tick(t_curv);

    // Energy penalty
    Eigen::MatrixXd T_q = AT_solver.solve(Eigen::MatrixXd(A_ex_bcd.transpose())).transpose();
    Eigen::MatrixXd T_qnx = T_q * N_x;
    Eigen::MatrixXd T_qny = T_q * N_y;

    // Sparse Q_E -> Q_E_sp
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(num_points * 9); 

    Eigen::Matrix3d block;
    block << 1, 1, 1,
             1, 4.0/3.0, 3.0/2.0,
             1, 3.0/2.0, 9.0/5.0;

    for (int i = 0; i < num_points; ++i) {
        int base = i * 3;
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                triplets.emplace_back(base + row, base + col, block(row, col));
            }
        }
    }
    
    Eigen::SparseMatrix<double> Q_E_sp(num_points * 3, num_points * 3);
    Q_E_sp.setFromTriplets(triplets.begin(), triplets.end());
    Q_E_sp.makeCompressed();
    
    Eigen::VectorXd T_c_p_x = T_c * p_x;
    Eigen::VectorXd T_c_p_y = T_c * p_y;
    Eigen::VectorXd T_q_p_x = T_q * p_x;
    Eigen::VectorXd T_q_p_y = T_q * p_y;

    tick(t_energy);

    // Objective function formulation (keep diagonal data as vectors)
    Eigen::VectorXd P_xx_vec = (curvatures_sq_den.array() * y_dot_sq.array()).matrix();
    Eigen::VectorXd P_yy_vec = (curvatures_sq_den.array() * x_dot_sq.array()).matrix();
    Eigen::VectorXd P_xy_vec = (curvatures_sq_den.array() * x_dot_y_dot.array()).matrix();

    Eigen::MatrixXd T_nx = T_c * N_x;
    Eigen::MatrixXd T_ny = T_c * N_y;

    Eigen::MatrixXd H_x = T_nx.transpose() * P_xx_vec.asDiagonal() * T_nx;
    Eigen::MatrixXd H_xy = T_ny.transpose() * P_xy_vec.asDiagonal() * T_nx;
    Eigen::MatrixXd H_y = T_ny.transpose() * P_yy_vec.asDiagonal() * T_ny;
    Eigen::MatrixXd H_E = T_qnx.transpose() * Q_E_sp * T_qnx + T_qny.transpose() * Q_E_sp * T_qny;
    Eigen::MatrixXd H_k = H_x + H_xy + H_y;
    Eigen::MatrixXd H = H_k + gamma * H_E;
    H = (H + H.transpose()) * 0.5;
    // double regularization = 1e-6;
    // H += regularization * Eigen::MatrixXd::Identity(H.rows(), H.cols());

    Eigen::VectorXd f_x  = 2.0 * (T_nx.transpose() * (P_xx_vec.asDiagonal() * T_c_p_x));
    Eigen::VectorXd f_y  = 2.0 * (T_ny.transpose() * (P_yy_vec.asDiagonal() * T_c_p_y));
    Eigen::VectorXd f_xy = (T_ny.transpose() * (P_xy_vec.asDiagonal() * T_c_p_x)) +
                           (T_nx.transpose() * (P_xy_vec.asDiagonal() * T_c_p_y));
    Eigen::VectorXd f_k = f_x + f_xy + f_y;
    Eigen::VectorXd f_E = 2.0 * (T_qnx.transpose() * (Q_E_sp * T_q_p_x)) + 2.0 * (T_qny.transpose() * (Q_E_sp * T_q_p_y));
    Eigen::VectorXd f = f_k + gamma * f_E;
    tick(t_H);

    // Constraint function formulation
    Eigen::VectorXd P_x = curvatures_den.array() * y_dot.array();
    Eigen::VectorXd P_y = curvatures_den.array() * x_dot.array();

    Eigen::MatrixXd E_curvature = P_y.asDiagonal() * T_ny - P_x.asDiagonal() * T_nx;
    Eigen::VectorXd curvature_reference = P_y.asDiagonal() * T_c_p_y - P_x.asDiagonal() * T_c_p_x;

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
    tick(t_constraints);

    // Debug: detailed matrix-build timing
    // ROS_INFO_STREAM("[Timing][Build] triplet=" << t_triplet
    //     << ", factor=" << t_factor
    //     << ", Tc=" << t_Tc
    //     << ", normal_triplet=" << t_normal_triplet
    //     << ", normal_set=" << t_normal_set
    //     << ", solve_px=" << t_solve_px
    //     << ", solve_py=" << t_solve_py
    //     << ", curv=" << t_curv
    //     << ", energy=" << t_energy
    //     << ", H/f=" << t_H
    //     << ", constraints=" << t_constraints);

    accum_stage(wall_time_matrix_build);

    // Use upper triangular part of H
    Eigen::MatrixXd H_upper = H.triangularView<Eigen::Upper>();

    // Log sparsity information for profiling

    // Use OSQP settings as usual for dense matrix optimization
    OSQPSettings *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    osqp_set_default_settings(settings);

    // rho: 0.5 adap, 0.7 uni?
    settings->rho = 0.7;
    settings->alpha = 1.6;
    settings->verbose = 0;
    settings->warm_start = 1;
    // settings->eps_abs = 1e-2;
    // settings->eps_rel = 1e-2;
    // settings->adaptive_rho_interval = 10;
    // settings->polish = 1;
    // settings->polish_refine_iter = 3;
    
    // OSQP data
    OSQPData *data = (OSQPData *)c_malloc(sizeof(OSQPData));
    data->n = H_upper.rows();
    data->m = E.rows();
    data->P = denseToCSC(H_upper);
    data->q = f.data();
    data->A = denseToCSC(E);
    data->l = l.data();
    data->u = u.data();
    accum_stage(wall_time_csc);

    // Setup workspace
    OSQPWorkspace *work;
    osqp_setup(&work, data, settings);
    accum_stage(wall_time_setup);

    // Solve the problem
    ros::WallTime wall_solve_start = ros::WallTime::now();
    osqp_solve(work);
    wall_time_solve += (ros::WallTime::now() - wall_solve_start).toSec();
    wall_stage_start = ros::WallTime::now();

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
    accum_stage(wall_time_cleanup);

    double total = (ros::WallTime::now() - wall_getalpha_start).toSec();
    double accounted = wall_time_matrix_build + wall_time_csc + wall_time_setup + wall_time_solve + wall_time_cleanup;
    double other = total - accounted;
    // ROS_INFO_STREAM("[Timing][OSQP] total=" << total
    //                 << " | matrix_build=" << wall_time_matrix_build
    //                 << ", csc=" << wall_time_csc
    //                 << ", setup=" << wall_time_setup
    //                 << ", solve=" << wall_time_solve
    //                 << ", cleanup=" << wall_time_cleanup
    //                 << ", other=" << other);

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
    if (path.size() < 2 || velocity_profile.size() < 2) {
        throw std::invalid_argument("Path and velocity profile must have at least 2 points.");
    }

    double total_time = 0.0;
    const double min_velocity = 1e-6;

    for (size_t i = 1; i < path.size(); ++i) {
        double distance = (path[i] - path[i - 1]).norm();
        
        double avg_velocity = (velocity_profile[i] + velocity_profile[i - 1]) / 2.0;

        if (avg_velocity < min_velocity) {
            avg_velocity = min_velocity;
        }

        total_time += distance / avg_velocity;
    }

    return total_time;
}

std::vector<Eigen::Vector2d> convertPx2Meter(const std::vector<Eigen::Vector2d>& path) {
    std::vector<Eigen::Vector2d> new_path = path;
    for (auto& point : new_path) {
        point.x() *= Constant::MAP_RESOLUTION;
        point.y() *= Constant::MAP_RESOLUTION;
    }

    return new_path;
}

} // namespace my_global_planner
