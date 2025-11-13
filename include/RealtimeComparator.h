#pragma once
#include "Pathfinder.h"
#include <vector>
#include <string>

struct ComparatorEntry {
    std::string name;
    PathfindingResult result;
};

class RealtimeComparator {
public:
    RealtimeComparator();
    ~RealtimeComparator() = default;

    // Run all available algorithms and return their results
    std::vector<ComparatorEntry> runAll(const Grid& grid, const Point& start, const Point& goal, bool eightDir = false);
};
