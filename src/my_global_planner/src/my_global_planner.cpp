// my_global_planner.cpp
#include "my_global_planner.h"
#include "a_star.h"
#include "constant.h"

#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <cmath>

// Register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(my_global_planner::MyGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace my_global_planner {

MyGlobalPlanner::MyGlobalPlanner() : a_star_planner(nullptr) {
}

MyGlobalPlanner::~MyGlobalPlanner() {
    if (a_star_planner) {
        delete a_star_planner;
        a_star_planner = nullptr;
    }
}

MyGlobalPlanner::MyGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros);
}

void MyGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    costmap = costmap_ros->getCostmap();
    a_star_planner = new basic_planner::AStarPlanner(costmap);
    ros::NodeHandle private_nh("~/" + name);
    plan_publisher_a_star = private_nh.advertise<nav_msgs::Path>("plan/a_star_plan", 1);
    plan_publisher_pose_smooth = private_nh.advertise<nav_msgs::Path>("plan/post_smooth", 1);
    plan_publisher_uniform = private_nh.advertise<nav_msgs::Path>("plan/uniform", 1);
    plan_publisher_bnd_L = private_nh.advertise<nav_msgs::Path>("plan/bnd_L", 1);
    plan_publisher_bnd_R = private_nh.advertise<nav_msgs::Path>("plan/bnd_R", 1);
    ROS_INFO("MyGlobalPlanner initialized successfully.");
}

bool MyGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                               const geometry_msgs::PoseStamped& goal,
                               std::vector<geometry_msgs::PoseStamped>& plan) {
                                
    std::string frame_id = "map";
    plan.clear();

    // Convert start and goal point: (Real) World -> Map
    unsigned int start_x, start_y, goal_x, goal_y;
    if (!costmap->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
        !costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
        ROS_ERROR("Start or goal coordinates are out of bounds.");
        return false;
    }


    // Find initial Path: A* Planning
    ROS_INFO("Find path using A*.");
    std::vector<geometry_msgs::PoseStamped> a_star_plan;

    bool a_star_success = a_star_planner->makePlan(start_x, start_y, goal_x, goal_y, a_star_plan);
    updateOrientatoin(a_star_plan);
    nav_msgs::Path a_star_plan_world = producePathMapToWorld(a_star_plan);

    if (a_star_success) {
        ROS_INFO("Initial A* planning is successful.");

        publishPath(plan_publisher_a_star, a_star_plan_world, frame_id);
        plan = a_star_plan_world.poses;
    } else {
        ROS_WARN("Initial A* planning failed.");
    }

    // Post smoothing
    std::vector<geometry_msgs::PoseStamped> post_smooth_plan = postSmooth(a_star_plan);
    nav_msgs::Path post_smooth_plan_world = producePathMapToWorld(post_smooth_plan);
    publishPath(plan_publisher_pose_smooth, post_smooth_plan_world, frame_id);
    plan = post_smooth_plan_world.poses;

    // Uniform path Spacing
    std::vector<geometry_msgs::PoseStamped> uniform_plan = uniformSpacing(post_smooth_plan);
    nav_msgs::Path uniform_plan_world = producePathMapToWorld(uniform_plan);
    publishPath(plan_publisher_uniform, uniform_plan_world, frame_id);
    // Adaptive path spacing

    // Get boundary
    Boundaries bnd;
    getBoundary(bnd, uniform_plan);
    nav_msgs::Path left_bnd_world = producePathMapToWorld(bnd.left_bnd.points);
    nav_msgs::Path right_bnd_world = producePathMapToWorld(bnd.right_bnd.points);
    publishPath(plan_publisher_bnd_L, left_bnd_world, frame_id);
    publishPath(plan_publisher_bnd_R, right_bnd_world, frame_id);

    return a_star_success;
}

void MyGlobalPlanner::updateOrientatoin(std::vector<geometry_msgs::PoseStamped>& plan) {
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

void MyGlobalPlanner::publishPath(const ros::Publisher& publisher, nav_msgs::Path& path_msg, const std::string& frame_id) {
    path_msg.header.frame_id = frame_id;
    path_msg.header.stamp = ros::Time::now();    
    publisher.publish(path_msg);
}

nav_msgs::Path MyGlobalPlanner::producePathMapToWorld (std::vector<geometry_msgs::PoseStamped>& plan) {
    nav_msgs::Path new_plan;
    geometry_msgs::PoseStamped new_pose;
    double world_x, world_y;
    double next_world_x, next_world_y;

    new_pose.header.frame_id = "map";
    new_pose.header.stamp = ros::Time::now();

    if (plan.empty()) {
        ROS_WARN("Fail to produce the path: Empty");
        return new_plan;
    }

    for (size_t i  = 0; i < plan.size() - 1; ++i) {

        costmap->mapToWorld(plan[i].pose.position.x, plan[i].pose.position.y, world_x, world_y);
        new_pose.pose.position.x = world_x;
        new_pose.pose.position.y = world_y;
        new_pose.pose.position.z = 0;
        new_pose.pose.orientation = plan[i].pose.orientation;

        new_plan.poses.push_back(new_pose);
    }

    return new_plan;
}

bool MyGlobalPlanner::isClearLine(geometry_msgs::PoseStamped& start_pose, geometry_msgs::PoseStamped& end_pose) {
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

std::vector<geometry_msgs::PoseStamped> MyGlobalPlanner::postSmooth (std::vector<geometry_msgs::PoseStamped>& plan) {
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

void MyGlobalPlanner::getBoundary(Boundaries& boundaries, const std::vector<geometry_msgs::PoseStamped>& plan) {
    std::size_t n = plan.size();
    boundaries.left_bnd.points.resize(n);
    boundaries.left_bnd.distances.resize(n);
    boundaries.right_bnd.points.resize(n);
    boundaries.right_bnd.distances.resize(n);

    for (size_t i = 0; i < n; ++i) {
        const auto& pose = plan[i];
        double yaw = tf::getYaw(pose.pose.orientation);
        
        double perp_yaw_left = yaw + M_PI_2;
        double perp_yaw_right = yaw - M_PI_2;

        double left_dx = cos(perp_yaw_left);
        double left_dy = sin(perp_yaw_left);
        double right_dx = cos(perp_yaw_right);
        double right_dy = sin(perp_yaw_right);

        boundaries.left_bnd.distances[i] = checkBoundary(pose, left_dx, left_dy);
        boundaries.right_bnd.distances[i] = checkBoundary(pose, right_dx, right_dy);
    }
    
    // constrain deviation change
    constrainDeviationChange(boundaries.left_bnd.distances);
    constrainDeviationChange(boundaries.right_bnd.distances);

    // Construct pose (only for visualizing in Rviz)
    for (size_t i = 0; i < n; ++i) {
        const auto& pose = plan[i];
        double yaw = tf::getYaw(pose.pose.orientation);
        
        double perp_yaw_left = yaw + M_PI_2;
        double perp_yaw_right = yaw - M_PI_2;

        double left_dx = cos(perp_yaw_left);
        double left_dy = sin(perp_yaw_left);
        double right_dx = cos(perp_yaw_right);
        double right_dy = sin(perp_yaw_right);

        boundaries.left_bnd.points[i] = createBoundaryPoint(pose, left_dx, left_dy, boundaries.left_bnd.distances[i]);
        boundaries.right_bnd.points[i] = createBoundaryPoint(pose, right_dx, right_dy, boundaries.right_bnd.distances[i]);
    }
    updateOrientatoin(boundaries.left_bnd.points);
    updateOrientatoin(boundaries.right_bnd.points);
}

double MyGlobalPlanner::checkBoundary(const geometry_msgs::PoseStamped& pose, double dx, double dy) {
    int x = pose.pose.position.x;
    int y = pose.pose.position.y;
    
    for (int d = 0; d < Constant::DEV_MAX; ++d) {
        int nx = x + std::round(dx * d);
        int ny = y + std::round(dy * d);
        
        if (nx < 0 || ny < 0 || nx >= costmap->getSizeInCellsX() || ny >= costmap->getSizeInCellsY()) {
            ROS_ERROR("Coordinates (%d, %d) are out of costmap bounds.", nx, ny);
        }

        if (costmap->getCost(nx, ny) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
            return d; // Return the distance at which an obstacle is encountered
        }
    }

    return Constant::DEV_MAX; // Return the maximum distance if no obstacles are encountered
}

void MyGlobalPlanner::constrainDeviationChange(std::vector<float>& dev_list) {
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

geometry_msgs::PoseStamped MyGlobalPlanner::createBoundaryPoint(const geometry_msgs::PoseStamped& pose, double dx, double dy, double distance) {
    geometry_msgs::PoseStamped boundary_pose;
    boundary_pose.pose.position.x = pose.pose.position.x + dx * distance;
    boundary_pose.pose.position.y = pose.pose.position.y + dy * distance;
    boundary_pose.pose.position.z = 0;

    return boundary_pose;
}

std::vector<geometry_msgs::PoseStamped> MyGlobalPlanner::uniformSpacing(const std::vector<geometry_msgs::PoseStamped>& plan) {
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

std::vector<geometry_msgs::PoseStamped> MyGlobalPlanner::adaptiveSpacing(const std::vector<geometry_msgs::PoseStamped>& plan) {

}

} // namespace my_global_planner