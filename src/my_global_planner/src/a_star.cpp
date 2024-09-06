// a_star.cpp
#include "a_star.h"
#include "constant.h"

namespace basic_planner {

    // Node
    Node::Node() : x(0), y(0), g_cost(std::numeric_limits<float>::max()), h_cost(0), f_cost(std::numeric_limits<float>::max()), parent(nullptr) {}
    
    Node::Node(unsigned int x, unsigned int y, float g_cost=1000, Node* parent=nullptr)
        : x(x), y(y), parent(parent), g_cost(g_cost), h_cost(0), f_cost(std::numeric_limits<float>::max()) {}


    // A* Planner
    AStarPlanner::AStarPlanner() {
        size_x_in_cells = my_global_planner::costmap->getSizeInCellsX();
        size_y_in_cells = my_global_planner::costmap->getSizeInCellsY();
    }

    bool AStarPlanner::makePlan(const unsigned int start_x, const unsigned int start_y,
                                const unsigned int goal_x, const unsigned int goal_y,
                                std::vector<geometry_msgs::PoseStamped>& plan) {

        Node* start_node = getInitNode(start_x, start_y);
        Node* goal_node = getNode(goal_x, goal_y);

        start_node->g_cost = 0;
        start_node->f_cost = start_node->g_cost + calculateH(start_x, start_y, goal_x, goal_y);
        open_set_.push(start_node);

        while (!open_set_.empty()) {
            Node* current = open_set_.top();
            open_set_.pop();

            if (current == goal_node) {
                // Reconstruction
                Node* path_node = current;
                geometry_msgs::PoseStamped pose;
                while (path_node != nullptr) {
                    pose.pose.position.x = path_node->x;
                    pose.pose.position.y = path_node->y;

                    plan.push_back(pose);
                    path_node = path_node->parent;
                }
                std::reverse(plan.begin(), plan.end());
                return true;
            }

            for (auto& neighbor_pair : getNeighbors(*current)) {
                Node* neighbor = neighbor_pair.first;
                float cost = neighbor_pair.second;
                float tentative_g_cost = current->g_cost + cost;

                if (tentative_g_cost < neighbor->g_cost) {
                    neighbor->parent = current;
                    neighbor->g_cost = tentative_g_cost;
                    neighbor->h_cost = calculateH(neighbor->x, neighbor->y, goal_x, goal_y);
                    neighbor->f_cost = neighbor->g_cost + neighbor->h_cost;
                    open_set_.push(neighbor);
                }
            }
        }
        return false;
    }

    Node* AStarPlanner::getInitNode(unsigned int x, unsigned int y) {
        unsigned int index = y * size_x_in_cells + x;
        if (all_nodes_.find(index) == all_nodes_.end()) {
            all_nodes_[index] = Node(x, y, 0.0f, nullptr);
        }
        return &all_nodes_[index];
    }

    Node* AStarPlanner::getNode(unsigned int x, unsigned int y) {
        unsigned int index = y * size_x_in_cells + x;
        if (all_nodes_.find(index) == all_nodes_.end()) {
            all_nodes_[index] = Node(x, y, 1000.0f, nullptr);
        }
        return &all_nodes_[index];
    }

    std::vector<std::pair<Node*, float>> AStarPlanner::getNeighbors(Node& current) {
        std::vector<std::pair<Node*, float>> neighbors;
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;
                int nx = current.x + dx;
                int ny = current.y + dy;
                if (my_global_planner::costmap->getCost(nx, ny) < Constant::OBSTACLE_INFLATION_INIT) {
                    float cost = std::sqrt(dx * dx + dy * dy);
                    neighbors.push_back(std::make_pair(getNode(nx, ny), cost));
                }
            }
        }
        return neighbors;
    }

    double AStarPlanner::calculateH(unsigned int x, unsigned int y, unsigned int goal_x, unsigned int goal_y) {
        // Using Euclidiean distance as heuristic
        int dx = static_cast<int>(x) - static_cast<int>(goal_x);
        int dy = static_cast<int>(y) - static_cast<int>(goal_y);
        return std::sqrt(dx * dx + dy * dy);
    }
}