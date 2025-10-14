#pragma once
#include "Pathfinder.h"
#include "Point.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <cmath>

struct AStarNode {
    Point position;
    float g, h, f;
    std::shared_ptr<AStarNode> parent;
    
    AStarNode(Point pos, float g_cost = 0, float h_cost = 0, std::shared_ptr<AStarNode> p = nullptr)
        : position(pos), g(g_cost), h(h_cost), f(g_cost + h_cost), parent(p) {}
};

struct NodeCompare {
    bool operator()(const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) const {
        // MinGW-safe float comparison
        float diff = a->f - b->f;
        if (diff > 0.001f || diff < -0.001f) return a->f > b->f;
        return a->h > b->h;
    }
};

class AStarPathfinder : public Pathfinder {
public:
    AStarPathfinder();
    virtual ~AStarPathfinder() = default;
    
    PathfindingResult findPath(const Grid& grid, const Point& start, const Point& goal) override;
    void reset() override;
    void setHeuristic(const std::string& heuristicType) override;
    void setMovementType(bool eightDirectional) override;
    
    std::string getName() const override { return "A*"; }
    int getTotalNodesExpanded() const override { return totalNodesExpanded_; }
    std::chrono::milliseconds getTotalPlanningTime() const override { return totalPlanningTime_; }

private:
    float calculateHeuristic(const Point& from, const Point& to) const;
    std::vector<Point> reconstructPath(std::shared_ptr<AStarNode> goalNode) const;
    void cleanup();
    
    std::string heuristicType_;
    bool eightDirectional_;
    int totalNodesExpanded_;
    std::chrono::milliseconds totalPlanningTime_;
    Point goalPosition_;
    
    std::priority_queue<std::shared_ptr<AStarNode>, std::vector<std::shared_ptr<AStarNode>>, NodeCompare> openList_;
    std::unordered_set<Point> closedList_;
    std::unordered_map<Point, std::shared_ptr<AStarNode>> allNodes_;
};
