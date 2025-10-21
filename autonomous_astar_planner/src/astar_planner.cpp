#include "astar_planner.h"
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <memory>

struct Node {
    Point position;
    std::shared_ptr<Node> parent;
    double g_cost;
    double h_cost;
    double f_cost;
    
    Node(Point pos, std::shared_ptr<Node> parent = nullptr, 
         double g = 0, double h = 0)
        : position(pos), parent(parent), g_cost(g), h_cost(h), f_cost(g + h) {}
    
    bool operator>(const Node& other) const {
        return f_cost > other.f_cost;
    }
};

AStarPlanner::AStarPlanner(std::shared_ptr<GridMap> map) : map_(map) {}

double AStarPlanner::euclideanHeuristic(const Point& a, const Point& b) const {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

double AStarPlanner::manhattanHeuristic(const Point& a, const Point& b) const {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

std::vector<Point> AStarPlanner::getNeighbors(const Point& current) const {
    std::vector<Point> neighbors;
    const std::vector<Point> directions = {
        {0, 1}, {0, -1}, {-1, 0}, {1, 0},
        {-1, 1}, {1, 1}, {-1, -1}, {1, -1}
    };
    
    for (const auto& dir : directions) {
        Point neighbor(current.x + dir.x, current.y + dir.y);
        if (map_->isTraversable(neighbor)) {
            neighbors.push_back(neighbor);
        }
    }
    
    return neighbors;
}

double AStarPlanner::getMoveCost(const Point& from, const Point& to) const {
    int dx = std::abs(from.x - to.x);
    int dy = std::abs(from.y - to.y);
    
    if (dx == 1 && dy == 1) {
        return 1.414;
    } else {
        return 1.0;
    }
}

std::vector<Point> AStarPlanner::reconstructPath(std::shared_ptr<Node> goal_node) {
    std::vector<Point> path;
    auto current = goal_node;
    
    while (current != nullptr) {
        path.push_back(current->position);
        current = current->parent;
    }
    
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Point> AStarPlanner::findPath(const Point& start, const Point& goal) {
    if (!map_->isTraversable(start) || !map_->isTraversable(goal)) {
        std::cout << "Start or goal position is not traversable!" << std::endl;
        return {};
    }
    
    if (start == goal) {
        return {start};
    }
    
    auto nodeCompare = [](const std::shared_ptr<Node>& a, 
                         const std::shared_ptr<Node>& b) {
        return a->f_cost > b->f_cost;
    };
    std::priority_queue<std::shared_ptr<Node>, 
                       std::vector<std::shared_ptr<Node>>, 
                       decltype(nodeCompare)> open_list(nodeCompare);
    
    std::unordered_map<Point, double> visited_costs;
    
    auto start_node = std::make_shared<Node>(start);
    start_node->h_cost = euclideanHeuristic(start, goal);
    start_node->f_cost = start_node->g_cost + start_node->h_cost;
    
    open_list.push(start_node);
    visited_costs[start] = 0.0;
    
    int iterations = 0;
    const int MAX_ITERATIONS = 10000;
    
    while (!open_list.empty() && iterations < MAX_ITERATIONS) {
        iterations++;
        
        auto current_node = open_list.top();
        open_list.pop();
        
        if (current_node->position == goal) {
            std::cout << "Path found! Iterations: " << iterations << std::endl;
            return reconstructPath(current_node);
        }
        
        for (const auto& neighbor_pos : getNeighbors(current_node->position)) {
            double new_g_cost = current_node->g_cost + 
                              getMoveCost(current_node->position, neighbor_pos);
            
            auto visited_it = visited_costs.find(neighbor_pos);
            if (visited_it == visited_costs.end() || new_g_cost < visited_it->second) {
                visited_costs[neighbor_pos] = new_g_cost;
                
                double h_cost = euclideanHeuristic(neighbor_pos, goal);
                auto neighbor_node = std::make_shared<Node>(
                    neighbor_pos, current_node, new_g_cost, h_cost);
                
                open_list.push(neighbor_node);
            }
        }
    }
    
    std::cout << "Path not found after " << iterations << " iterations!" << std::endl;
    return {};
}