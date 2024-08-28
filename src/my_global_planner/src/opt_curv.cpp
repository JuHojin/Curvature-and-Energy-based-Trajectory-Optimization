// opt_curv.cpp
#include "my_global_planner.h"
#include "opt_curv.h"
#include "constant.h"
#include "path_func.h"
#include "cubic_spline.h"

#include <ros/ros.h>
#include <Eigen/Dense>
#include <osqp.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <limits>
#include <tf/tf.h>
// #include "spline/src/spline.h"

namespace my_global_planner {

std::vector<Eigen::Vector2d> optimize_min_curvature(std::vector<Eigen::Vector2d> path, const size_t max_iter) {
    
    std::vector<Eigen::Vector2d> new_path;
    Eigen::VectorXd alpha;

    // std::ostringstream oss;
    // for (size_t i = 0; i < path.size(); ++i) {
    //     oss << " original path: (" << path[i].x() << ", " << path[i].y() << ")\n";
    // }
    // ROS_INFO_STREAM(oss.str());
    
    for (int i = 0; i < max_iter; ++i) {
        ROS_INFO("Optimzation Step [%d]", i+1);
        if (i != 0) {
            path = uniformSpacingVector2d(new_path);
        }

        Eigen::MatrixXd A = getSystemMatrix(path);
        std::vector<Eigen::Vector2d> normal_vectors = getNormalVectors(path);

        // Boundaries
        Boundaries Boundaries = getBoundaries(path, normal_vectors);
        std::vector<geometry_msgs::PoseStamped> left_bnd = convertVector2dToPoseStamped(Boundaries.left_bnd.points);
        std::vector<geometry_msgs::PoseStamped> right_bnd = convertVector2dToPoseStamped(Boundaries.right_bnd.points);
        nav_msgs::Path left_bnd_world = producePathMapToWorld(left_bnd);
        nav_msgs::Path right_bnd_world = producePathMapToWorld(right_bnd);
        publishPath(plan_publisher_bnd_L, left_bnd_world);
        publishPath(plan_publisher_bnd_R, right_bnd_world);

         // Calculate start angle
        Eigen::Vector2d start_segment = path[1] - path[0];
        double start_angle = std::atan2(start_segment.y(), start_segment.x());

        // Calculate end angle
        Eigen::Vector2d end_segment = path[path.size() - 1] - path[path.size() - 2];
        double end_angle = std::atan2(end_segment.y(), end_segment.x());

        // Alpha
        auto result = getAlphaMinCurvature(path, Boundaries.left_bnd.distances, Boundaries.right_bnd.distances, normal_vectors, A, Constant::CURVATURE_BOUND, start_angle, end_angle);
        alpha = result.first;
        int success = result.second;

        if (success == OSQP_SOLVED) {
            new_path = getPathMinCurvature(path, normal_vectors, alpha);
            std::vector<geometry_msgs::PoseStamped> new_path_tmp = convertVector2dToPoseStamped(new_path);
            nav_msgs::Path new_path_world = producePathMapToWorld(new_path_tmp);
            publishPath(plan_publisher_opt, new_path_world);
        } else {
            ROS_WARN("Optimization Fail at step[%d]",i+1);
            break;
        }
        
    }

    return new_path;
}

Eigen::MatrixXd getSystemMatrix(const std::vector<Eigen::Vector2d>& path) {
    int numPoints = path.size();
    int numSplines = numPoints - 1;
    Eigen::VectorXd pointDist(numSplines);

    // Calculate distances between consecutive points
    for (int i = 0; i < numSplines; ++i) {
        pointDist(i) = (path[i + 1] - path[i]).norm();
    }

    Eigen::VectorXd scaling(numSplines - 1);
    for (int i = 0; i < numSplines - 1; ++i) {
        scaling(i) = pointDist(i) / pointDist(i + 1);
    }

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(numSplines * 4, numSplines * 4);

    // Base matrix for constraints, repeated in A
    Eigen::MatrixXd baseA(4, 8);
    baseA   <<  1, 0, 0, 0, 0, 0, 0, 0,
                1, 1, 1, 1, 0, 0, 0, 0,
                0, 1, 2, 3, 0, -1, 0, 0,
                0, 0, 2, 6, 0, 0, -2, 0;

    // Construct the system matrix with appropriate scaling for continuity constraints
    for (int i = 0; i < numSplines; ++i) {
        int j = i * 4;

        if (i < numSplines - 1) {
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
    A(numSplines * 4 - 2, 1) = 1;

    // End point heading constraint
    A(numSplines * 4 - 1, numSplines * 4 - 4 + 1) = 1;
    A(numSplines * 4 - 1, numSplines * 4 - 4 + 2) = 2;
    A(numSplines * 4 - 1, numSplines * 4 - 4 + 3) = 3;

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

Eigen::VectorXd getCurvatureUsingSpline(const std::vector<Eigen::Vector2d>& path, const Eigen::MatrixXd& A, double start_angle, double end_angle) {
    int numPoints = path.size();
    int numSplines = numPoints - 1;

    if (std::isnan(start_angle)) {
        Eigen::Vector2d firstSegment = path[1] - path[0];
        start_angle = std::atan2(firstSegment.y(), firstSegment.x());
    }
    if (std::isnan(end_angle)) {
        Eigen::Vector2d lastSegment = path[numPoints - 1] - path[numPoints - 2];
        end_angle = std::atan2(lastSegment.y(), lastSegment.x());
    }

    // Construct extraction matrices
    Eigen::MatrixXd A_ex_b = Eigen::MatrixXd::Zero(numPoints, numSplines * 4);
    for (int i = 0; i < numSplines; ++i) {
        A_ex_b(i, i * 4 + 1) = 1;
    }
    A_ex_b.bottomRightCorner(1, 4) << 0, 1, 2, 3;

    Eigen::MatrixXd A_ex_c = Eigen::MatrixXd::Zero(numPoints, numSplines * 4);
    for (int i = 0; i < numSplines; ++i) {
        A_ex_c(i, i * 4 + 2) = 2;
    }
    A_ex_c.bottomRightCorner(1, 4) << 0, 0, 2, 6;

    Eigen::MatrixXd A_inv = A.inverse();
    Eigen::MatrixXd T_c = A_ex_c * A_inv;

    // Construct reference path matrix
    Eigen::VectorXd q_x = Eigen::VectorXd::Zero(numSplines * 4);
    Eigen::VectorXd q_y = Eigen::VectorXd::Zero(numSplines * 4);
    for (int i = 0; i < numSplines; ++i) {
        int j = i * 4;
        q_x.segment(j, 2) << path[i].x(), path[i + 1].x();
        q_y.segment(j, 2) << path[i].y(), path[i + 1].y();
    }
    q_x(numSplines * 4 - 2) = std::cos(start_angle);
    q_y(numSplines * 4 - 2) = std::sin(start_angle);
    q_x(numSplines * 4 - 1) = std::cos(end_angle);
    q_y(numSplines * 4 - 1) = std::sin(end_angle);

    // Calculate derivatives
    Eigen::VectorXd x_dot = A_ex_b * A_inv * q_x;
    Eigen::VectorXd y_dot = A_ex_b * A_inv * q_y;

    Eigen::VectorXd x_dot_sq = x_dot.array().square();
    Eigen::VectorXd y_dot_sq = y_dot.array().square();

    // Calculate curvature denominator
    Eigen::VectorXd curv_den = (x_dot_sq + y_dot_sq).array().pow(1.5);
    Eigen::VectorXd curvatures_den = (1.0 / curv_den.array()).matrix();

    Eigen::VectorXd Q_x = curvatures_den.asDiagonal() * y_dot;
    Eigen::VectorXd Q_y = curvatures_den.asDiagonal() * x_dot;

    Eigen::VectorXd curvature = Q_y.asDiagonal() * T_c * q_y - Q_x.asDiagonal() * T_c * q_x;

    return curvature;
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

Boundaries getBoundaries(const std::vector<Eigen::Vector2d>& path, const std::vector<Eigen::Vector2d>& normal_vectors) {
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

        boundaries.left_bnd.distances[i] = checkBoundary(point, left_dx, left_dy);
        boundaries.right_bnd.distances[i] = checkBoundary(point, right_dx, right_dy);
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

double checkBoundary(const Eigen::Vector2d& point, double dx, double dy) {
    int x = point.x();
    int y = point.y();
    
    int last_valid_d = Constant::DEV_MAX;

    for (int d = Constant::DEV_MAX; d > 0; --d) {
        int nx = x + std::round(dx * d);
        int ny = y + std::round(dy * d);
        
        if (nx < 0 || ny < 0 || nx >= costmap->getSizeInCellsX() || ny >= costmap->getSizeInCellsY()) {
            // If we hit a boundary, return the last valid distance
            return last_valid_d;
        }

        if (costmap->getCost(nx, ny) >= Constant::OBSTACLE_INFLATION) {
            // If we hit an obstacle, reduce the distance and continue
            last_valid_d = d - 1;
        } else {
            // If no obstacle is found, return the last valid distance
            return last_valid_d;
        }
    }

    return last_valid_d; // Return the final valid distance if no obstacles are encountered within the loop
}

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

//         if (isClearLine(x, y, nx,z)) {
//             // If we hit an obstacle, reduce the distance and continue
//             last_valid_d = d - 1;
//         } else {
//             // If no obstacle is found, return the last valid distance
//             return last_valid_d;
//         }
//     }

//     return last_valid_d; // Return the final valid distance if no obstacles are encountered within the loop
// }

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

csc* eigenToCSC(const Eigen::SparseMatrix<double>& mat) {
    // Count nonzeros in the upper triangular part
    int nonzeros = 0;
    for (int k = 0; k < mat.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(mat, k); it; ++it) {
            if (it.row() <= it.col()) {
                nonzeros++;
            }
        }
    }

    c_int* row_indices = (c_int*)c_malloc(nonzeros * sizeof(c_int));
    c_int* col_pointers = (c_int*)c_malloc((mat.cols() + 1) * sizeof(c_int));
    c_float* values = (c_float*)c_malloc(nonzeros * sizeof(c_float));

    int idx = 0;
    col_pointers[0] = 0;
    for (int k = 0; k < mat.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(mat, k); it; ++it) {
            if (it.row() <= it.col()) { // Only upper triangular part
                values[idx] = it.value();
                row_indices[idx] = it.row();
                idx++;
            }
        }
        col_pointers[k + 1] = idx;
    }

    return csc_matrix(mat.rows(), mat.cols(), nonzeros, values, row_indices, col_pointers);
}


std::pair<Eigen::VectorXd, int> getAlphaMinCurvature(const std::vector<Eigen::Vector2d>& path,
                                     std::vector<double> left_dev_bnd,
                                     std::vector<double> right_dev_bnd,
                                     const std::vector<Eigen::Vector2d>& normal_vectors,
                                     const Eigen::MatrixXd& A,
                                     double curvature_bnd,
                                     double start_angle, double end_angle) {
    std::size_t num_points = path.size();
    std::size_t num_splines = num_points - 1;

    // Construct extraction matrices : b component extraction
    Eigen::MatrixXd A_ex_b = Eigen::MatrixXd::Zero(num_points, num_splines * 4);
    for (int i = 0; i < num_splines; ++i) {
        A_ex_b(i, i * 4 + 1) = 1;
    }
    A_ex_b.row(num_points - 1).tail(4) << 0, 1, 2, 3;

    // Construct extraction matrices : c component extraction
    Eigen::MatrixXd A_ex_c = Eigen::MatrixXd::Zero(num_points, num_splines * 4);
    for (int i = 0; i < num_splines; ++i) {
        A_ex_c(i, i * 4 + 2) = 2;
    }
    A_ex_c.row(num_points - 1).tail(4) << 0, 0, 2, 6;

    // Eigen::MatrixXd A_inv = A.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXd A_inv = A.inverse();
    Eigen::MatrixXd T_c = A_ex_c * A_inv;

    // Construct normal vector matrix
    Eigen::MatrixXd M_x = Eigen::MatrixXd::Zero(num_splines * 4, num_points);
    Eigen::MatrixXd M_y = Eigen::MatrixXd::Zero(num_splines * 4, num_points);

    for (int i = 0; i < num_splines; ++i) {
        int j = i * 4;
        const auto& normal = normal_vectors[i];

        if (i < num_points - 1) {
            const auto& normal_next = normal_vectors[i + 1];

            M_x(j, i) = normal.x();
            M_x(j + 1, i + 1) = normal_next.x();
            M_y(j, i) = normal.y();
            M_y(j + 1, i + 1) = normal_next.y();
        } else {
            M_x(j, i) = normal.x();
            M_x(j + 1, 0) = normal_vectors[0].x();

            M_y(j, i) = normal.y();
            M_y(j + 1, 0) = normal_vectors[0].y();
        }
    }

    // Construct reference path vector
    Eigen::VectorXd q_x = Eigen::VectorXd::Zero(num_splines * 4);
    Eigen::VectorXd q_y = Eigen::VectorXd::Zero(num_splines * 4);

    for (int i = 0; i < num_splines; ++i) {
        int j = i * 4;
        const auto& point = path[i];

        if (i < num_points - 1) {
            const auto& point_next = path[i + 1];

            q_x(j) = point.x();
            q_x(j + 1) = point_next.x();

            q_y(j) = point.y();
            q_y(j + 1) = point_next.y();
        } else {
            q_x(j) = point.x();
            q_x(j + 1) = path[0].x();

            q_y(j) = point.y();
            q_y(j + 1) = path[0].y();
        }
    }

    q_x(num_splines * 4 - 2) = std::cos(start_angle);
    q_y(num_splines * 4 - 2) = std::sin(start_angle);
    q_x(num_splines * 4 - 1) = std::cos(end_angle);
    q_y(num_splines * 4 - 1) = std::sin(end_angle);

    // Calculate derivatives: x', y', x'y'
    Eigen::VectorXd x_dot = A_ex_b * A_inv * q_x;
    Eigen::VectorXd y_dot = A_ex_b * A_inv * q_y;
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

    // Objective function formulation
    Eigen::MatrixXd P_xx = (curvatures_sq_den.array() * y_dot_sq.array()).matrix().asDiagonal();
    Eigen::MatrixXd P_yy = (curvatures_sq_den.array() * x_dot_sq.array()).matrix().asDiagonal();
    Eigen::MatrixXd P_xy = (curvatures_sq_den.array() * x_dot_y_dot.array()).matrix().asDiagonal();

    Eigen::MatrixXd T_nx = T_c * M_x;
    Eigen::MatrixXd T_ny = T_c * M_y;

    Eigen::MatrixXd H_x = T_nx.transpose() * P_xx * T_nx;
    Eigen::MatrixXd H_xy = T_ny.transpose() * P_xy * T_nx;
    Eigen::MatrixXd H_y = T_ny.transpose() * P_yy * T_ny;
    Eigen::MatrixXd H = H_x + H_xy + H_y;
    H = (H + H.transpose()) * 0.5;
    // double regularization = 1e-6;
    // H += regularization * Eigen::MatrixXd::Identity(H.rows(), H.cols());

    Eigen::VectorXd f_x = 2 * q_x.transpose() * T_c.transpose() * P_xx * T_nx;
    Eigen::VectorXd f_y = 2 * q_y.transpose() * T_c.transpose() * P_yy * T_ny;
    Eigen::VectorXd f_xy = q_x.transpose() * T_c.transpose() * P_xy * T_ny + q_y.transpose() * T_c.transpose() * P_xy * T_nx;
    Eigen::VectorXd f = f_x + f_xy + f_y;

    // Constraint function formulation
    Eigen::VectorXd Q_x = curvatures_den.array() * y_dot.array();
    Eigen::VectorXd Q_y = curvatures_den.array() * x_dot.array();

    Eigen::MatrixXd E_curvature = Q_y.asDiagonal() * T_ny - Q_x.asDiagonal() * T_nx;
    Eigen::VectorXd curvature_reference = Q_y.asDiagonal() * T_c * q_y - Q_x.asDiagonal() * T_c * q_x;

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

    // Convert Eigen matrices/vectors to string for ROS_INFO
    std::stringstream ss_E, ss_u, ss_l;
    // ss_E << E;
    ss_u << (E_curvature.inverse() * curv_constraints_ge).transpose();
    ss_l << (E_curvature.inverse() * curv_constraints_le).transpose();

    // ROS_INFO_STREAM to output the contents
    // ROS_INFO_STREAM("Matrix E:\n" << ss_E.str());
    ROS_INFO_STREAM("Vector u:\n" << ss_u.str());
    ROS_INFO_STREAM("Vector l:\n" << ss_l.str());

    // Check convexity by inspecting the eigenvalues of H
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(H);
    ROS_INFO_STREAM("Eigenvalues of H: " << eigen_solver.eigenvalues().transpose());

    // Solve QP
    Eigen::SparseMatrix<double> H_sparse = H.sparseView();
    Eigen::SparseMatrix<double> E_sparse = E.sparseView();

    // OSQP problem settings
    OSQPSettings *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    osqp_set_default_settings(settings);
    settings->rho = 0.1;
    settings->alpha = 1.6;

    // OSQP data
    OSQPData *data = (OSQPData *)c_malloc(sizeof(OSQPData));
    data->n = H.rows();
    data->m = E.rows();
    data->P = eigenToCSC(H_sparse);
    data->q = f.data();
    data->A = eigenToCSC(E_sparse);
    data->l = l.data();
    data->u = u.data();

    // Setup workspace
    OSQPWorkspace *work;
    osqp_setup(&work, data, settings);

    // Solve the problem
    osqp_solve(work);

    // Extract the solution
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
        
        std::ostringstream oss;
        oss << "Iteration " << i << ":\n"
            << "  path[" << i << "]: (" << path[i].x() << ", " << path[i].y() << ")\n"
            << "  normal_vectors[" << i << "]: (" << normal_vectors[i].x() << ", " << normal_vectors[i].y() << ")\n"
            << "  alpha[" << i << "]: " << alpha(i) << "\n"
            << "  new_path[" << i << "]: (" << new_path[i].x() << ", " << new_path[i].y() << ")";

        ROS_INFO_STREAM(oss.str());
    }

    return new_path;
}


} // namespace my_global_planner

