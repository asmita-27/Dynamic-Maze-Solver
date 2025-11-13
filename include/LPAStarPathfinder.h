#pragma once
#include "Pathfinder.h"
#include "Point.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>

struct LPAKey {
    float k1, k2;
    
    LPAKey(float k1 = 0, float k2 = 0) : k1(k1), k2(k2) {}
    
    bool operator<(const LPAKey& other) const {
        float diff1 = k1 - other.k1;
        if (diff1 < -0.001f || diff1 > 0.001f) return k1 < other.k1;
        return k2 < other.k2;
    }
    
    bool operator>(const LPAKey& other) const {
        float diff1 = k1 - other.k1;
        if (diff1 < -0.001f || diff1 > 0.001f) return k1 > other.k1;
        return k2 > other.k2;
    }
};

struct LPANode {
    Point position;
    float g, rhs;
    LPAKey key;
    
    LPANode(Point pos = Point()) : position(pos), g(1000000.0f), rhs(1000000.0f) {}
};

struct LPACompare {
    bool operator()(const std::pair<LPAKey, Point>& a, const std::pair<LPAKey, Point>& b) const {
        return a.first > b.first;
    }
};

class LPAStarPathfinder : public Pathfinder {
public:
    LPAStarPathfinder();
    virtual ~LPAStarPathfinder() = default;
    
    PathfindingResult findPath(const Grid& grid, const Point& start, const Point& goal) override;
    PathfindingResult repairPath(const Grid& grid, const Point& currentPos) override;
    void updateGrid(const Grid& grid, const std::vector<Point>& changedCells) override;
    void reset() override;
    void setHeuristic(const std::string& heuristicType) override;
    void setMovementType(bool eightDirectional) override;
    
    std::string getName() const override { return "LPA*"; }
    int getTotalNodesExpanded() const override { return totalNodesExpanded_; }
    std::chrono::milliseconds getTotalPlanningTime() const override { return totalPlanningTime_; }

private:
    float calculateHeuristic(const Point& from, const Point& to) const;
    LPAKey calculateKey(const Point& pos) const;
    void updateVertex(const Point& pos, const Grid& grid);
    void computeShortestPath(const Grid& grid);
    std::vector<Point> extractPath(const Grid& grid) const;
    float getCost(const Point& from, const Point& to, const Grid& grid) const;
    void initialize(const Grid& grid, const Point& start, const Point& goal);
    
    std::string heuristicType_;
    bool eightDirectional_;
    int totalNodesExpanded_;
    std::chrono::milliseconds totalPlanningTime_;
    
    Point startPos_, goalPos_;
    std::unordered_map<Point, LPANode> nodes_;
    std::priority_queue<std::pair<LPAKey, Point>, 
                       std::vector<std::pair<LPAKey, Point>>, 
                       LPACompare> openQueue_;
    std::unordered_set<Point> inQueue_;
    bool initialized_;
};
