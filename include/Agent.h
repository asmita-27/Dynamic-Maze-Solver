#pragma once
#include "Grid.h"
#include "Pathfinder.h"
#include "Point.h"
#include <memory>
#include <vector>

class Agent {
public:
    Agent(const std::string& pathfinderType = "AStar");
    ~Agent() = default;
    
    // Setup
    void setStart(const Point& start) { start_ = start; position_ = start; needsReplanning_ = true; }
    void setGoal(const Point& goal) { goal_ = goal; needsReplanning_ = true; }
    void setMovementSpeed(int stepsPerSecond);
    void setPathfinder(const std::string& pathfinderType);
    
    // Navigation
    void planPath(const Grid& grid);
    void update(Grid& grid);
    void reset();
    void stepForward(Grid& grid);  // Manual step forward
    
    // Movement mode control
    void setManualMode(bool manual) { manualMode_ = manual; }
    bool isManualMode() const { return manualMode_; }
    
    // Status
    Point getPosition() const { return position_; }
    Point getStart() const { return start_; }
    Point getGoal() const { return goal_; }
    const std::vector<Point>& getCurrentPath() const { return currentPath_; }
    bool hasPath() const { return !currentPath_.empty() && pathIndex_ < currentPath_.size(); }
    bool hasReachedGoal() const { return position_ == goal_; }
    
    // Statistics
    const PathfindingResult& getLastResult() const { return lastResult_; }
    std::string getPathfinderName() const;
    int getTotalNodesExpanded() const;
    std::chrono::milliseconds getTotalPlanningTime() const;
    void forceReplanning() { needsReplanning_ = true; }

private:
    Point position_;
    Point start_;
    Point goal_;
    std::vector<Point> currentPath_;
    size_t pathIndex_;
    bool needsReplanning_;
    
    std::unique_ptr<Pathfinder> pathfinder_;
    PathfindingResult lastResult_;
    
    unsigned long lastMoveTime_;
    unsigned long moveInterval_;
    bool manualMode_;
    
    bool needsReplanning(const Grid& grid) const;
    void moveToNext();
};
