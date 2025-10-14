#pragma once
#include "Point.h"
#include <vector>
#include <string>
#include <queue>

enum class CellType {
    FREE,
    OBSTACLE,
    START,
    GOAL
};

struct GridEvent {
    unsigned long timestamp;
    Point position;
    CellType newType;
    std::string description;
};

class Grid {
public:
    Grid(int width, int height, bool eightDirectional = false);
    
    // Basic operations
    bool inBounds(const Point& pos) const;
    bool isFree(const Point& pos) const;
    float getCost(const Point& pos) const;
    CellType getCellType(const Point& pos) const;
    void setCell(const Point& pos, CellType type);
    
    // Navigation
    std::vector<Point> getNeighbors(const Point& pos) const;
    
    // Properties
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    bool isEightDirectional() const { return eightDirectional_; }
    
    // Dynamic features
    void generateRandomObstacles(float density);
    void processScheduledEvents();
    void processRandomEvents(float probability);
    void toggleObstacle(const Point& pos);
    
    // Change tracking
    bool hasChanges() const { return changesMade_; }
    void clearChanges() { changesMade_ = false; }
    const std::vector<GridEvent>& getRecentEvents() const { return recentEvents_; }
    
    void clear();

private:
    int width_, height_;
    bool eightDirectional_;
    std::vector<std::vector<CellType>> cells_;
    
    // Dynamic events
    std::queue<GridEvent> scheduledEvents_;
    std::vector<GridEvent> recentEvents_;
    mutable bool changesMade_;
    unsigned long lastRandomEvent_;
    
    void loadScheduledEvents();
};
