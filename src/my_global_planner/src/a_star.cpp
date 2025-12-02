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
        nodes_.resize(size_x_in_cells * size_y_in_cells);
        closed_.assign(size_x_in_cells * size_y_in_cells, 0);
        touched_.reserve(4096);
        for (unsigned int y = 0; y < size_y_in_cells; ++y) {
            for (unsigned int x = 0; x < size_x_in_cells; ++x) {
                Node& n = nodes_[idx(x, y)];
                n.x = x;
                n.y = y;
                n.g_cost = std::numeric_limits<float>::infinity();
                n.h_cost = 0.0f;
                n.f_cost = std::numeric_limits<float>::infinity();
                n.parent = nullptr;
            }
        }
    }

    bool AStarPlanner::makePlan(const unsigned int start_x, const unsigned int start_y,
                                const unsigned int goal_x, const unsigned int goal_y,
                                std::vector<geometry_msgs::PoseStamped>& plan) {

        // reset structures
        resetTouched();
        open_set_ = std::priority_queue<Node*, std::vector<Node*>, CompareNode>();

        Node* start_node = getNode(start_x, start_y);
        Node* goal_node = getNode(goal_x, goal_y);

        start_node->g_cost = 0.0f;
        start_node->h_cost = calculateH(start_x, start_y, goal_x, goal_y);
        start_node->f_cost = start_node->g_cost + start_node->h_cost;
        start_node->parent = nullptr;
        open_set_.push(start_node);
        touched_.push_back(idx(start_x, start_y));

        while (!open_set_.empty()) {
            Node* current = open_set_.top();
            open_set_.pop();

            unsigned int current_idx = idx(current->x, current->y);
            if (closed_[current_idx]) {
                continue;
            }
            closed_[current_idx] = 1;
            touched_.push_back(current_idx);

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
                    touched_.push_back(idx(neighbor->x, neighbor->y));
                }
            }
        }
        return false;
    }

    Node* AStarPlanner::getNode(unsigned int x, unsigned int y) {
        unsigned int index = idx(x, y);
        Node& n = nodes_[index];
        return &n;
    }

    std::vector<std::pair<Node*, float>> AStarPlanner::getNeighbors(Node& current) {
        std::vector<std::pair<Node*, float>> neighbors;
        neighbors.reserve(8);
        static const int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
        static const int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
        static const float cost_straight = 1.0f;
        static const float cost_diag = 1.41421356f;

        for (int i = 0; i < 8; ++i) {
            int nx = static_cast<int>(current.x) + dx[i];
            int ny = static_cast<int>(current.y) + dy[i];
            if (nx < 0 || ny < 0 || nx >= static_cast<int>(size_x_in_cells) || ny >= static_cast<int>(size_y_in_cells)) {
                continue;
            }
            unsigned int nux = static_cast<unsigned int>(nx);
            unsigned int nuy = static_cast<unsigned int>(ny);
            if (my_global_planner::costmap->getCost(nux, nuy) >= Constant::OBSTACLE_INFLATION_INIT) {
                continue;
            }
            float step_cost = (dx[i] == 0 || dy[i] == 0) ? cost_straight : cost_diag;
            neighbors.emplace_back(getNode(nux, nuy), step_cost);
        }
        return neighbors;
    }

    double AStarPlanner::calculateH(unsigned int x, unsigned int y, unsigned int goal_x, unsigned int goal_y) {
        // Octile distance (admissible for 8-connected grid)
        int dx = std::abs(static_cast<int>(x) - static_cast<int>(goal_x));
        int dy = std::abs(static_cast<int>(y) - static_cast<int>(goal_y));
        int maxd = std::max(dx, dy);
        int mind = std::min(dx, dy);
        constexpr double kOctile = 1.41421356237 - 1.0;
        return static_cast<double>(maxd) + kOctile * static_cast<double>(mind);
    }

    void AStarPlanner::resetTouched() {
        for (unsigned int idx_val : touched_) {
            nodes_[idx_val].g_cost = std::numeric_limits<float>::infinity();
            nodes_[idx_val].h_cost = 0.0f;
            nodes_[idx_val].f_cost = std::numeric_limits<float>::infinity();
            nodes_[idx_val].parent = nullptr;
            closed_[idx_val] = 0;
        }
        touched_.clear();
    }
} 
