// opt_curv.cpp
#include "my_global_planner.h"
#include "opt_curv.h"
#include "constant.h"
#include "path_func.h"

#include <ros/ros.h>
#include <Eigen/Dense>
#include <osqp.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <limits>
#include <tf/tf.h>

namespace my_global_planner {

std::vector<Eigen::Vector2d> optimize_min_curvature(const std::vector<Eigen::Vector2d>& path, const size_t max_iter) {
    Eigen::MatrixXd A;
    std::vector<Eigen::Vector2d> normal_vectors;

    Boundaries Boundaries;

    std::vector<Eigen::Vector2d> new_path;

    for (int i = 0; i < max_iter; ++i) {
        ROS_INFO("Optimzation Step [%d]", i+1);

        updateSystemMatrix(path, A);
        updateNormalVectors(path, normal_vectors);

        // Boundaries
        updateBoundaries(Boundaries, path, normal_vectors);
        std::vector<geometry_msgs::PoseStamped> left_bnd = convertVector2dToPoseStamped(Boundaries.left_bnd.points);
        std::vector<geometry_msgs::PoseStamped> right_bnd = convertVector2dToPoseStamped(Boundaries.right_bnd.points);
        nav_msgs::Path left_bnd_world = producePathMapToWorld(left_bnd);
        nav_msgs::Path right_bnd_world = producePathMapToWorld(right_bnd);
        publishPath(plan_publisher_bnd_L, left_bnd_world);
        publishPath(plan_publisher_bnd_R, right_bnd_world);
    }

    return new_path;
}

void updateSystemMatrix(const std::vector<Eigen::Vector2d>& path, Eigen::MatrixXd& A) {
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

    A = Eigen::MatrixXd::Zero(numSplines * 4, numSplines * 4);

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
}

void updateNormalVectors(const std::vector<Eigen::Vector2d>& path, std::vector<Eigen::Vector2d>& normal_vectors) {
    if (path.size() < 2) {
        ROS_WARN("Cannot compute normal vectors: at least two points are required.");
        return;
    }

    normal_vectors.resize(path.size());

    for (size_t i = 0; i < path.size(); ++i) {
        Eigen::Vector2d normal;

        if (i == 0) {
            // For start point
            double dx = path[i + 1].x() - path[i].x();
            double dy = path[i + 1].y() - path[i].y();
            normal = Eigen::Vector2d(-dy, dx);
        } else if (i == path.size() - 1) {
            // For goal point
            double dx = path[i].x() - path[i - 1].x();
            double dy = path[i].y() - path[i - 1].y();
            normal = Eigen::Vector2d(-dy, dx);
        } else {
            // For intermediate points
            double dx = path[i + 1].x() - path[i - 1].x();
            double dy = path[i + 1].y() - path[i - 1].y();
            normal = Eigen::Vector2d(-dy, dx);
        }

        normal.normalize();
        normal_vectors[i] = normal;
    }
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

void updateBoundaries(Boundaries& boundaries, const std::vector<Eigen::Vector2d>& path, const std::vector<Eigen::Vector2d>& normal_vectors) {
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
}

double checkBoundary(const Eigen::Vector2d& point, double dx, double dy) {
    int x = point.x();
    int y = point.y();
    
    for (int d = 0; d < Constant::DEV_MAX; ++d) {
        int nx = x + std::round(dx * d);
        int ny = y + std::round(dy * d);
        
        if (nx < 0 || ny < 0 || nx >= costmap->getSizeInCellsX() || ny >= costmap->getSizeInCellsY()) {
            return d; // Return the distance at which boundary is reached
        }

        if (costmap->getCost(nx, ny) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
            return d; // Return the distance at which an obstacle is encountered
        }
    }// Return the distance at which an obstacle is encountered or boundary is reached
    return Constant::DEV_MAX; // Return the maximum distance if no obstacles are encountered
}

void constrainDeviationChange(std::vector<float>& dev_list) {
    // Constrain forward direction
    float dev = dev_list[0];

    for (int i = 1; i < dev_list.size(); ++i) {
        float next_dev = dev + Constant::DEV_DOT;
        if (next_dev < dev_list[i]) {
            dev_list[i] = next_dev;
        }
        dev = dev_list[i];
    }

    // Constrain backward direction
    dev = dev_list.back();
    for (int i = dev_list.size() - 2; i >= 0; --i) {
        float prev_dev = dev + Constant::DEV_DOT;
        if (prev_dev < dev_list[i]) {
            dev_list[i] = prev_dev;
        }
        dev = dev_list[i];
    }
}

Eigen::Vector2d createBoundaryPoint(const Eigen::Vector2d& point, double dx, double dy, double distance) {
    return Eigen::Vector2d(point.x() + dx * distance, point.y() + dy * distance);
}

// Eigen::VectorXd getAlphaMinCurvature(const std::vector<Eigen::Vector2d>& path,
//                                                     const std::vector<double>& dev_bnd,
//                                                     const std::vector<Eigen::Vector2d>& normal_vectors,
//                                                     const Eigen::MatrixXd& A,
//                                                     double curvature_bnd,
//                                                     double start_angle, double end_angle) {
//     std::size_t num_points = path.size();
//     std::size_t num_splines = num_points - 1;

//     // Matrix and vector setups for the optimization problem
//     Eigen::MatrixXd P, Q, E;
//     Eigen::VectorXd f, k;

//     // Construct extraction matrices : b component extraction
//     Eigen::MatrixXd A_ex_b = Eigen::MatrixXd::Zero(num_points, num_splines * 4);
//     for (int i = 0; i < num_splines; ++i) {
//         A_ex_b[i, i * 4 + 1] = 1;
//     }
//     A_ex_b.row(num_points - 1).tail(4) << 0, 1, 2, 3;

//     // Construct extraction matrices : c component extraction
//     Eigen::MatrixXd A_ex_c = Eigen::MatrixXd::Zero(num_points, num_splines * 4);
//     for (int i = 0; i < num_splines; ++i) {
//         A_ex_c(i, i * 4 + 2) = 2;
//     }
//     A_ex_c.row(num_points - 1).tail(4) << 0, 0, 2, 6;

//     // Calculate inverse of A matrix
//     Eigen::MatrixXd A_inv = A.inverse();

//     // Calculate T_c matrix
//     Eigen::MatrixXd T_c = A_ex_c * A_inv;

//     // Construct normal vector matrix
//     Eigen::MatrixXd M_x = Eigen::MatrixXd::Zero(num_splines * 4, num_points);
//     Eigen::MatrixXd M_y = Eigen::MatrixXd::Zero(num_splines * 4, num_points);

//     for (int i = 0; i < num_splines; ++i) {
//         int j = i * 4;
//         const auto& normal = normal_vectors[i];

//         if (i < num_points - 1) {
//             const auto& normal_next = normal_vectors[i + 1];

//             M_x(j, i) = normal.x();
//             M_x(j + 1, i + 1) = normal_next(0);
//             M_y(j, i) = normal.y();
//             M_y(j + 1, i + 1) = normal_next(1);
//         } else {
//             M_x(j, i) = normal.x();
//             M_x(j + 1, 0) = normal_vectors[0](0);

//             M_y(j, i) = normal.y();
//             M_y(j + 1, 0) = normal_vectors[0](1);
//         }
//     }

//     // Construct reference path matrix
//     Eigen::VectorXd q_x = Eigen::VectorXd::Zero(num_splines * 4);
//     Eigen::VectorXd q_y = Eigen::VectorXd::Zero(num_splines * 4);

//     for (int i = 0; i < num_splines; ++i) {
//         int j = i * 4;
//         const auto& point = path[i];

//         if (i < num_points - 1) {
//             const auto& point_next = path[i + 1];

//             q_x(j) = point(0);
//             q_x(j + 1) = point_next(0);

//             q_y(j) = point(1);
//             q_y(j + 1) = point_next(1);
//         } else {
//             q_x(j) = point(0);
//             q_x(j + 1) = path[0](0);

//             q_y(j) = point(1);
//             q_y(j + 1) = path[0](1);
//         }
//     }

//     q_x(num_splines * 4 - 2) = std::cos(start_angle);
//     q_y(num_splines * 4 - 2) = std::sin(start_angle);
//     q_x(num_splines * 4 - 1) = std::cos(end_angle);
//     q_y(num_splines * 4 - 1) = std::sin(end_angle);

//     // Calculate derivatives: x', y', x'y'
//     Eigen::VectorXd x_dot = A_ex_b * A_inv * q_x;
//     Eigen::VectorXd y_dot = A_ex_b * A_inv * q_y;
//     Eigen::VectorXd x_dot_y_dot = -2 * x_dot.array() * y_dot.array();

//     // Square of derivatives
//     Eigen::VectorXd x_dot_sq = x_dot.array().square();
//     Eigen::VectorXd y_dot_sq = y_dot.array().square();

//     // Curvature denominator calculations
//     Eigen::VectorXd curv_den = (x_dot_sq + y_dot_sq).array().pow(1.5);
//     Eigen::VectorXd curvatures_den = curv_den.array().inverse();
//     curvatures_den = curvatures_den.unaryExpr([](double v) { return std::isfinite(v) ? v : 0.0; }); // Check infinity
//     Eigen::VectorXd curvatures_sq_den = curvatures_den.array().square();

//     // Objective function formulation
//     Eigen::MatrixXd P_xx = (curvatures_sq_den.asDiagonal() * Eigen::MatrixXd::Identity(num_points, num_points)) * (y_dot_sq.asDiagonal() * Eigen::MatrixXd::Identity(num_points, num_points));
//     Eigen::MatrixXd P_yy = (curvatures_sq_den.asDiagonal() * Eigen::MatrixXd::Identity(num_points, num_points)) * (x_dot_sq.asDiagonal() * Eigen::MatrixXd::Identity(num_points, num_points));
//     Eigen::MatrixXd P_xy = (curvatures_sq_den.asDiagonal() * Eigen::MatrixXd::Identity(num_points, num_points)) * (x_dot_y_dot.asDiagonal() * Eigen::MatrixXd::Identity(num_points, num_points));

//     Eigen::MatrixXd T_nx = T_c * M_x;
//     Eigen::MatrixXd T_ny = T_c * M_y;

//     Eigen::MatrixXd H_x = T_nx.transpose() * P_xx * T_nx;
//     Eigen::MatrixXd H_xy = T_ny.transpose() * P_xy * T_nx;
//     Eigen::MatrixXd H_y = T_ny.transpose() * P_yy * T_ny;
//     Eigen::MatrixXd H = H_x + H_xy + H_y;
//     H = (H + H.transpose()) * 0.5;

//     Eigen::VectorXd f_x = 2 * q_x.transpose() * T_c.transpose() * P_xx * T_nx;
//     Eigen::VectorXd f_y = 2 * q_y.transpose() * T_c.transpose() * P_yy * T_ny;
//     Eigen::VectorXd f_xy = q_x.transpose() * T_c.transpose() * P_xy * T_ny + q_y.transpose() * T_c.transpose() * P_xy * T_nx;
//     Eigen::VectorXd f = f_x + f_xy + f_y;

//     // Constraint function formulation
//     Eigen::VectorXd Q_x = curvatures_den.asDiagonal() * y_dot;
//     Eigen::VectorXd Q_y = curvatures_den.asDiagonal() * x_dot;

//     Eigen::MatrixXd E_curvature = Q_y.asDiagonal() * T_ny - Q_x.asDiagonal() * T_nx;
//     Eigen::VectorXd curvature_reference = Q_y.asDiagonal() * T_c * q_y - Q_x.asDiagonal() * T_c * q_x;

//     Eigen::VectorXd constraints_ge = Eigen::VectorXd::Constant(num_points, curvature_bnd) - curvature_reference;
//     Eigen::VectorXd constraints_le = -constraints_ge;

//     // Boundary constraints
//     Eigen::VectorXd left_dev_bnd = Eigen::Map<const Eigen::VectorXd>(dev_bnd.data(), num_points);
//     Eigen::VectorXd right_dev_bnd = Eigen::VectorXd::Constant(num_points, 0.05);  // End points fixed
//     left_dev_bnd(0) = right_dev_bnd(0) = left_dev_bnd(num_points - 1) = right_dev_bnd(num_points - 1) = 0.05;

//     Eigen::MatrixXd E = Eigen::MatrixXd::Zero(2 * num_points, num_points);
//     E.topRows(num_points) = Eigen::MatrixXd::Identity(num_points, num_points);
//     E.bottomRows(num_points) = -E.topRows(num_points);

//     Eigen::VectorXd k(num_points * 2);
//     k.head(num_points) = left_dev_bnd;
//     k.tail(num_points) = right_dev_bnd;

//     // All constraints
//     E.conservativeResize(E.rows() + E_curvature.rows(), Eigen::NoChange);
//     E.bottomRows(E_curvature.rows()) = E_curvature;
//     k.conservativeResize(k.size() + constraints_ge.size() + constraints_le.size());
//     k.segment(num_points * 2, constraints_ge.size()) = constraints_ge;
//     k.segment(num_points * 2 + constraints_ge.size(), constraints_le.size()) = constraints_le;

//     // Sparse Matrices
//     Eigen::SparseMatrix<double> H_sparse = H.sparseView();
//     Eigen::SparseMatrix<double> E_sparse = E.sparseView();

//     // OSQP setup ...

//     // return alpha_min_curvature;
// }

} // namespace my_global_planner

