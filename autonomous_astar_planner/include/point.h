#ifndef POINT_H
#define POINT_H

#include <functional>

struct Point {
    int x, y;
    
    Point(int x = 0, int y = 0);
    bool operator==(const Point& other) const;
    bool operator<(const Point& other) const;
};

// 哈希函数特化
namespace std {
    template<>
    struct hash<Point> {
        size_t operator()(const Point& p) const;
    };
}

#endif