#pragma once
#include <functional>

struct Point {
    int x, y;
    
    Point(int x = 0, int y = 0) : x(x), y(y) {}
    
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
    
    bool operator!=(const Point& other) const {
        return !(*this == other);
    }
    
    Point operator+(const Point& other) const {
        return Point(x + other.x, y + other.y);
    }
    
    Point operator-(const Point& other) const {
        return Point(x - other.x, y - other.y);
    }
};

// Hash function for Point - MinGW compatible
namespace std {
    template<>
    struct hash<Point> {
        size_t operator()(const Point& p) const {
            return (size_t)(p.x * 1000 + p.y);
        }
    };
}
