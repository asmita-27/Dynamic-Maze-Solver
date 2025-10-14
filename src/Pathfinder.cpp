#include "Pathfinder.h"

bool Pathfinder::isPathValid(const std::vector<Point>& path, const Grid& grid) const {
    for (const Point& point : path) {
        if (!grid.isFree(point)) {
            return false;
        }
    }
    return true;
}
