#include "Grid.h"
#include <random>
#include <iostream>
#include <algorithm>
#include <windows.h>

Grid::Grid(int width, int height, bool eightDirectional) 
    : width_(width), height_(height), eightDirectional_(eightDirectional), 
      changesMade_(false), lastRandomEvent_(0) {
    
    cells_.resize(height_);
    for (int y = 0; y < height_; ++y) {
        cells_[y].resize(width_);
        for (int x = 0; x < width_; ++x) {
            cells_[y][x] = CellType::FREE;
        }
    }
    
    std::cout << "Grid initialized: " << width_ << "x" << height_ 
              << " (" << (eightDirectional_ ? "8" : "4") << "-directional)" << std::endl;
    
    loadScheduledEvents();
}

bool Grid::inBounds(const Point& pos) const {
    return pos.x >= 0 && pos.x < width_ && pos.y >= 0 && pos.y < height_;
}

bool Grid::isFree(const Point& pos) const {
    if (!inBounds(pos)) return false;
    CellType type = cells_[pos.y][pos.x];
    return type == CellType::FREE || type == CellType::START || type == CellType::GOAL;
}

float Grid::getCost(const Point& pos) const {
    if (!inBounds(pos)) return 1000.0f;
    return 1.0f; // Uniform cost for now
}

CellType Grid::getCellType(const Point& pos) const {
    if (!inBounds(pos)) return CellType::OBSTACLE;
    return cells_[pos.y][pos.x];
}

void Grid::setCell(const Point& pos, CellType type) {
    if (!inBounds(pos)) return;
    if (cells_[pos.y][pos.x] != type) {
        cells_[pos.y][pos.x] = type;
        changesMade_ = true;
    }
}

std::vector<Point> Grid::getNeighbors(const Point& pos) const {
    std::vector<Point> neighbors;
    
    if (eightDirectional_) {
        // 8-directional movement
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                if (dx == 0 && dy == 0) continue;
                Point neighbor(pos.x + dx, pos.y + dy);
                if (inBounds(neighbor)) {
                    neighbors.push_back(neighbor);
                }
            }
        }
    } else {
        // 4-directional movement
        Point directions[] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
        for (int i = 0; i < 4; i++) {
            Point neighbor = pos + directions[i];
            if (inBounds(neighbor)) {
                neighbors.push_back(neighbor);
            }
        }
    }
    
    return neighbors;
}

void Grid::generateRandomObstacles(float density) {
    if (density <= 0.0f) return;
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    
    int obstacleCount = 0;
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            Point pos(x, y);
            CellType currentType = cells_[y][x];
            
            if (currentType == CellType::FREE && dist(gen) < density) {
                cells_[y][x] = CellType::OBSTACLE;
                obstacleCount++;
            }
        }
    }
    
    if (obstacleCount > 0) {
        changesMade_ = true;
        std::cout << "Generated " << obstacleCount << " random obstacles (" 
                  << (density * 100.0f) << "% density)" << std::endl;
    }
}

void Grid::processScheduledEvents() {
    unsigned long currentTime = GetTickCount();
    bool eventsProcessed = false;
    
    while (!scheduledEvents_.empty()) {
        const GridEvent& event = scheduledEvents_.front();
        
        if (event.timestamp > currentTime) {
            break; // No more events ready to process
        }
        
        // Process the event
        if (inBounds(event.position)) {
            CellType currentType = cells_[event.position.y][event.position.x];
            if (currentType != CellType::START && currentType != CellType::GOAL) {
                cells_[event.position.y][event.position.x] = event.newType;
                
                // Add to recent events
                recentEvents_.push_back(event);
                if (recentEvents_.size() > 20) {
                    recentEvents_.erase(recentEvents_.begin());
                }
                
                eventsProcessed = true;
                changesMade_ = true;
                
                std::cout << "SCHEDULED: " << event.description 
                          << " at (" << event.position.x << "," << event.position.y << ")" << std::endl;
            }
        }
        
        scheduledEvents_.pop();
    }
}

void Grid::processRandomEvents(float probability) {
    unsigned long currentTime = GetTickCount();
    
    // Check random events every 2 seconds
    if (currentTime - lastRandomEvent_ < 2000) {
        return;
    }
    
    lastRandomEvent_ = currentTime;
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> probDist(0.0f, 1.0f);
    
    if (probDist(gen) > probability) {
        return; // No random event this time
    }
    
    // Generate random position (avoid edges)
    std::uniform_int_distribution<int> xDist(1, width_ - 2);
    std::uniform_int_distribution<int> yDist(1, height_ - 2);
    
    Point randomPos(xDist(gen), yDist(gen));
    CellType currentType = cells_[randomPos.y][randomPos.x];
    
    if (currentType == CellType::START || currentType == CellType::GOAL) {
        return; // Don't modify start/goal
    }
    
    // Toggle the cell
    bool wasObstacle = (currentType == CellType::OBSTACLE);
    CellType newType = wasObstacle ? CellType::FREE : CellType::OBSTACLE;
    cells_[randomPos.y][randomPos.x] = newType;
    
    // Log the event
    GridEvent event;
    event.timestamp = currentTime;
    event.position = randomPos;
    event.newType = newType;
    event.description = "Random event";
    
    recentEvents_.push_back(event);
    if (recentEvents_.size() > 20) {
        recentEvents_.erase(recentEvents_.begin());
    }
    
    changesMade_ = true;
    
    std::cout << "RANDOM: Cell (" << randomPos.x << "," << randomPos.y << ") " 
              << (wasObstacle ? "cleared" : "blocked") << std::endl;
}

void Grid::toggleObstacle(const Point& pos) {
    if (!inBounds(pos)) return;
    
    CellType currentType = cells_[pos.y][pos.x];
    if (currentType == CellType::START || currentType == CellType::GOAL) {
        return; // Don't modify start/goal cells
    }
    
    bool wasObstacle = (currentType == CellType::OBSTACLE);
    CellType newType = wasObstacle ? CellType::FREE : CellType::OBSTACLE;
    cells_[pos.y][pos.x] = newType;
    
    // Log the event
    GridEvent event;
    event.timestamp = GetTickCount();
    event.position = pos;
    event.newType = newType;
    event.description = wasObstacle ? "Manual obstacle removed" : "Manual obstacle added";
    
    recentEvents_.push_back(event);
    if (recentEvents_.size() > 20) {
        recentEvents_.erase(recentEvents_.begin());
    }
    
    changesMade_ = true;
    
    std::cout << "MANUAL: Cell (" << pos.x << "," << pos.y << ") " 
              << (wasObstacle ? "CLEARED" : "BLOCKED") 
              << " - Replanning triggered" << std::endl;
}

void Grid::clear() {
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            CellType currentType = cells_[y][x];
            if (currentType != CellType::START && currentType != CellType::GOAL) {
                cells_[y][x] = CellType::FREE;
            }
        }
    }
    changesMade_ = true;
    std::cout << "Grid cleared of obstacles" << std::endl;
}

void Grid::loadScheduledEvents() {
    unsigned long baseTime = GetTickCount();
    
    // Example: doors that open and close at specific times
    std::vector<Point> doorPositions = {
        Point(width_ / 2, height_ / 3),
        Point(width_ / 3, height_ / 2),
        Point(2 * width_ / 3, 2 * height_ / 3)
    };
    
    int timeOffset = 15000; // Start after 15 seconds
    
    for (const auto& doorPos : doorPositions) {
        // Door closes
        GridEvent closeEvent;
        closeEvent.timestamp = baseTime + timeOffset;
        closeEvent.position = doorPos;
        closeEvent.newType = CellType::OBSTACLE;
        closeEvent.description = "Door closes";
        scheduledEvents_.push(closeEvent);
        
        // Door opens later
        GridEvent openEvent;
        openEvent.timestamp = baseTime + timeOffset + 10000; // 10 seconds later
        openEvent.position = doorPos;
        openEvent.newType = CellType::FREE;
        openEvent.description = "Door opens";
        scheduledEvents_.push(openEvent);
        
        timeOffset += 8000; // Stagger the events
    }
    
    std::cout << "Loaded " << (doorPositions.size() * 2) << " scheduled door events" << std::endl;
}
