// a_star.h
#ifndef A_STAR_H
#define A_STAR_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <memory>
#include <vector>
#include <queue>
#include <map>
#include <unordered_map>

namespace basic_planner {

struct Node {
    unsigned int x, y;
    float g_cost;   // Cost from start node
    float h_cost;   // Heuristic cost to goal node
    float f_cost;   // Total cost (g_cost + h_cost)
    Node* parent;

    // Constructor
    Node();
    Node(unsigned int x, unsigned int y, float g_cost, Node* parent);

    // Comparison operator for priority queue (min heap)
    bool operator<(const Node& other) const {
        return f_cost > other.f_cost;
    }

    bool operator==(const Node& other) const {
        return x == other.x && y == other.y;
    }
};

struct CompareNode {
    bool operator()(const Node* a, const Node* b) const {
        return a->f_cost > b->f_cost;
    }
};

class AStarPlanner {
public:
    AStarPlanner(costmap_2d::Costmap2D* costmap);

    bool makePlan(const unsigned int start_x, const unsigned int start_y,
                  const unsigned int goal_x, const unsigned int goal_y,
                  std::vector<geometry_msgs::PoseStamped>& plan);

private:
    costmap_2d::Costmap2D* costmap_;
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> open_set_;
    std::unordered_map<unsigned int, Node> all_nodes_;
    unsigned int size_x_in_cells;
    unsigned int size_y_in_cells;

    Node* getInitNode(unsigned int x, unsigned int y);
    Node* getNode(unsigned int x, unsigned int y);
    std::vector<std::pair<Node*, float>> getNeighbors(Node& current);
    double calculateH(unsigned int x, unsigned int y, unsigned int goal_x, unsigned int goal_y);
};

} // namespace basic_planner

#endif // A_STAR_H