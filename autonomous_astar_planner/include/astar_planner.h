#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

#include "grid_map.h"
#include "point.h"
#include <memory>
#include <vector>

// 前向声明
struct Node;

class AStarPlanner {
private:
    std::shared_ptr<GridMap> map_;
    
public:
    AStarPlanner(std::shared_ptr<GridMap> map);
    std::vector<Point> findPath(const Point& start, const Point& goal);
    
private:
    double euclideanHeuristic(const Point& a, const Point& b) const;
    double manhattanHeuristic(const Point& a, const Point& b) const;
    std::vector<Point> getNeighbors(const Point& current) const;
    double getMoveCost(const Point& from, const Point& to) const;
    std::vector<Point> reconstructPath(std::shared_ptr<Node> goal_node);
};

#endif