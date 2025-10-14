#include "AStarPathfinder.h"
#include <cmath>
#include <algorithm>
#include <iostream>

AStarPathfinder::AStarPathfinder() 
    : heuristicType_("manhattan"), eightDirectional_(false), 
      totalNodesExpanded_(0), totalPlanningTime_(0),
      closedList_(), allNodes_() {
}

PathfindingResult AStarPathfinder::findPath(const Grid& grid, const Point& start, const Point& goal) {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    cleanup();
    PathfindingResult result;
    goalPosition_ = goal;
    
    if (!grid.isFree(start) || !grid.isFree(goal)) {
        std::cout << "A*: Start or goal is blocked!" << std::endl;
        result.planningTime = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - startTime);
        return result;
    }
    
    if (start == goal) {
        result.path = {start};
        result.success = true;
        result.pathCost = 0.0f;
        result.planningTime = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - startTime);
        return result;
    }
    
    // Initialize start node
    auto startNode = std::make_shared<AStarNode>(start, 0.0f, calculateHeuristic(start, goal));
    allNodes_[start] = startNode;
    openList_.push(startNode);
    
    int nodesExpanded = 0;
    
    while (!openList_.empty()) {
        auto current = openList_.top();
        openList_.pop();
        
        // Skip if already processed
        if (closedList_.count(current->position)) {
            continue;
        }
        
        closedList_.insert(current->position);
        nodesExpanded++;
        
        // Goal reached
        if (current->position == goal) {
            result.path = reconstructPath(current);
            result.success = true;
            result.nodesExpanded = nodesExpanded;
            result.pathCost = current->g;
            
            totalNodesExpanded_ += nodesExpanded;
            
            auto endTime = std::chrono::high_resolution_clock::now();
            result.planningTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
            totalPlanningTime_ += result.planningTime;
            
            std::cout << "A*: Path found! Length: " << result.path.size() 
                      << ", Cost: " << result.pathCost 
                      << ", Nodes: " << nodesExpanded
                      << ", Time: " << result.planningTime.count() << "ms" << std::endl;
            
            return result;
        }
        
        // Expand neighbors
        auto neighbors = grid.getNeighbors(current->position);
        for (const auto& neighborPos : neighbors) {
            if (!grid.isFree(neighborPos) || closedList_.count(neighborPos)) {
                continue;
            }
            
            float moveCost = grid.getCost(neighborPos);
            if (eightDirectional_) {
                // Adjust cost for diagonal movement
                Point diff = neighborPos - current->position;
                // Use simple comparison instead of abs() for MinGW compatibility
                int absDx = (diff.x < 0) ? -diff.x : diff.x;
                int absDy = (diff.y < 0) ? -diff.y : diff.y;
                if (absDx + absDy == 2) {
                    moveCost *= 1.414f; // sqrt(2)
                }
            }
            
            float tentativeG = current->g + moveCost;
            
            auto existing = allNodes_.find(neighborPos);
            std::shared_ptr<AStarNode> neighbor;
            
            if (existing != allNodes_.end()) {
                neighbor = existing->second;
                if (tentativeG >= neighbor->g) {
                    continue; // Not a better path
                }
            } else {
                neighbor = std::make_shared<AStarNode>(neighborPos);
                allNodes_[neighborPos] = neighbor;
            }
            
            // Update neighbor
            neighbor->g = tentativeG;
            neighbor->h = calculateHeuristic(neighborPos, goal);
            neighbor->f = neighbor->g + neighbor->h;
            neighbor->parent = current;
            
            openList_.push(neighbor);
        }
    }
    
    // No path found
    result.nodesExpanded = nodesExpanded;
    totalNodesExpanded_ += nodesExpanded;
    
    auto endTime = std::chrono::high_resolution_clock::now();
    result.planningTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    totalPlanningTime_ += result.planningTime;
    
    std::cout << "A*: No path found! Nodes: " << nodesExpanded 
              << ", Time: " << result.planningTime.count() << "ms" << std::endl;
    
    return result;
}

void AStarPathfinder::reset() {
    cleanup();
    totalNodesExpanded_ = 0;
    totalPlanningTime_ = std::chrono::milliseconds(0);
    std::cout << "A* pathfinder reset" << std::endl;
}

void AStarPathfinder::setHeuristic(const std::string& heuristicType) {
    heuristicType_ = heuristicType;
    std::cout << "A* heuristic set to: " << heuristicType_ << std::endl;
}

void AStarPathfinder::setMovementType(bool eightDirectional) {
    eightDirectional_ = eightDirectional;
    std::cout << "A* movement set to: " << (eightDirectional_ ? "8" : "4") << "-directional" << std::endl;
}

float AStarPathfinder::calculateHeuristic(const Point& from, const Point& to) const {
    // MinGW-safe absolute value calculation
    int dx = (to.x > from.x) ? (to.x - from.x) : (from.x - to.x);
    int dy = (to.y > from.y) ? (to.y - from.y) : (from.y - to.y);
    
    if (heuristicType_ == "manhattan") {
        return static_cast<float>(dx + dy);
    } else if (heuristicType_ == "euclidean") {
        return sqrtf(static_cast<float>(dx * dx + dy * dy));
    } else if (heuristicType_ == "octile" && eightDirectional_) {
        // Octile distance for 8-directional movement
        int maxDiff = (dx > dy) ? dx : dy;
        int minDiff = (dx < dy) ? dx : dy;
        return static_cast<float>(maxDiff) + 0.414f * static_cast<float>(minDiff);
    } else {
        // Default to Manhattan
        return static_cast<float>(dx + dy);
    }
}

std::vector<Point> AStarPathfinder::reconstructPath(std::shared_ptr<AStarNode> goalNode) const {
    std::vector<Point> path;
    auto current = goalNode;
    
    while (current != nullptr) {
        path.push_back(current->position);
        current = current->parent;
    }
    
    std::reverse(path.begin(), path.end());
    return path;
}

void AStarPathfinder::cleanup() {
    // Clear the priority queue
    while (!openList_.empty()) {
        openList_.pop();
    }
    
    closedList_.clear();
    allNodes_.clear();
}
