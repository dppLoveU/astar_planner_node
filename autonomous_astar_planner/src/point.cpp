#include "point.h"
#include <tuple>

Point::Point(int x, int y) : x(x), y(y) {}

bool Point::operator==(const Point& other) const {
    return x == other.x && y == other.y;
}

bool Point::operator<(const Point& other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
}

namespace std {
    size_t hash<Point>::operator()(const Point& p) const {
        return hash<int>()(p.x) ^ (hash<int>()(p.y) << 1);
    }
}