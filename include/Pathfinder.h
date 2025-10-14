#pragma once
#include "Grid.h"
#include "Point.h"
#include <vector>
#include <chrono>

struct PathfindingResult {
    std::vector<Point> path;
    int nodesExpanded;
    std::chrono::milliseconds planningTime;
    bool success;
    float pathCost;
    
    PathfindingResult() : nodesExpanded(0), planningTime(0), success(false), pathCost(0.0f) {}
};

class Pathfinder {
public:
    virtual ~Pathfinder() = default;
    
    virtual PathfindingResult findPath(const Grid& grid, const Point& start, const Point& goal) = 0;
    virtual void reset() = 0;
    virtual void setHeuristic(const std::string& heuristicType) = 0;
    virtual void setMovementType(bool eightDirectional) = 0;
    virtual std::string getName() const = 0;
    virtual int getTotalNodesExpanded() const = 0;
    virtual std::chrono::milliseconds getTotalPlanningTime() const = 0;
    virtual bool isPathValid(const std::vector<Point>& path, const Grid& grid) const;
    
    // For dynamic pathfinders
    virtual void updateGrid(const Grid& grid, const std::vector<Point>& changedCells) {}
    virtual PathfindingResult repairPath(const Grid& grid, const Point& currentPos) { 
        return findPath(grid, currentPos, Point()); 
    }
};
