#include "DynamicAStarPathfinder.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>

DynamicAStarPathfinder::DynamicAStarPathfinder() 
    : heuristicType_("manhattan"), eightDirectional_(false), 
      totalNodesExpanded_(0), totalPlanningTime_(0), 
    nodes_(), inQueue_(), initialized_(false), km_(0.0f) {
}

PathfindingResult DynamicAStarPathfinder::findPath(const Grid& grid, const Point& start, const Point& goal) {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    PathfindingResult result;
    
    if (!grid.isFree(start) || !grid.isFree(goal)) {
        std::cout << "D* Lite: Start or goal is blocked!" << std::endl;
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
    
    // Initialize if first time or goal changed
    if (!initialized_ || goalPos_ != goal) {
        initialize(grid, start, goal);
        computeShortestPath(grid);
        initialized_ = true;
    } else if (startPos_ != start) {
        // Start position changed - update km and heuristics
        float delta = calculateHeuristic(startPos_, start);
        km_ += delta;
        startPos_ = start;
        for (auto& pair : nodes_) {
            pair.second.key = calculateKey(pair.first);
        }
        computeShortestPath(grid);
    }
    
    // Extract path
    result.path = extractPath(grid);
    result.success = !result.path.empty();
    
    if (result.success) {
        result.pathCost = nodes_[start].g;
        result.nodesExpanded = static_cast<int>(nodes_.size());
        totalNodesExpanded_ += result.nodesExpanded;
        
        std::cout << "D* Lite: Path found! Length: " << result.path.size() 
                  << ", Cost: " << result.pathCost 
                  << ", Nodes: " << result.nodesExpanded;
    } else {
        std::cout << "D* Lite: No path found! Nodes: " << nodes_.size();
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    result.planningTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    totalPlanningTime_ += result.planningTime;
    
    std::cout << ", Time: " << result.planningTime.count() << "ms" << std::endl;
    
    return result;
}

PathfindingResult DynamicAStarPathfinder::repairPath(const Grid& grid, const Point& currentPos) {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    PathfindingResult result;
    
    if (!initialized_) {
        return findPath(grid, currentPos, goalPos_);
    }
    
    // Update start position
    // Increase km by heuristic distance between old and new start
    float delta = calculateHeuristic(startPos_, currentPos);
    km_ += delta;
    startPos_ = currentPos;
    
    // Recompute shortest path from new position
    computeShortestPath(grid);
    
    // Extract path
    result.path = extractPath(grid);
    result.success = !result.path.empty();
    
    if (result.success) {
        result.pathCost = nodes_[currentPos].g;
        result.nodesExpanded = static_cast<int>(nodes_.size());
        
        std::cout << "D* Lite Repair: Path repaired! Length: " << result.path.size() 
                  << ", Cost: " << result.pathCost;
    } else {
        std::cout << "D* Lite Repair: No path found!";
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    result.planningTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    totalPlanningTime_ += result.planningTime;
    
    std::cout << ", Time: " << result.planningTime.count() << "ms" << std::endl;
    
    return result;
}

void DynamicAStarPathfinder::updateGrid(const Grid& grid, const std::vector<Point>& changedCells) {
    if (!initialized_) return;
    
    std::cout << "D* Lite: Updating grid for " << changedCells.size() << " changed cells" << std::endl;
    
    // Update affected vertices
    for (const Point& pos : changedCells) {
        if (!grid.inBounds(pos)) continue;
        
        // Update this vertex and its neighbors
        updateVertex(pos, grid);
        
        auto neighbors = grid.getNeighbors(pos);
        for (const Point& neighbor : neighbors) {
            updateVertex(neighbor, grid);
        }
    }
}

void DynamicAStarPathfinder::reset() {
    nodes_.clear();
    while (!openQueue_.empty()) openQueue_.pop();
    inQueue_.clear();
    initialized_ = false;
    totalNodesExpanded_ = 0;
    totalPlanningTime_ = std::chrono::milliseconds(0);
    km_ = 0.0f;
    std::cout << "D* Lite pathfinder reset" << std::endl;
}

void DynamicAStarPathfinder::setHeuristic(const std::string& heuristicType) {
    heuristicType_ = heuristicType;
    std::cout << "D* Lite heuristic set to: " << heuristicType_ << std::endl;
}

void DynamicAStarPathfinder::setMovementType(bool eightDirectional) {
    eightDirectional_ = eightDirectional;
    std::cout << "D* Lite movement set to: " << (eightDirectional_ ? "8" : "4") << "-directional" << std::endl;
}

float DynamicAStarPathfinder::calculateHeuristic(const Point& from, const Point& to) const {
    // MinGW-safe absolute value calculation
    int dx = (to.x > from.x) ? (to.x - from.x) : (from.x - to.x);
    int dy = (to.y > from.y) ? (to.y - from.y) : (from.y - to.y);
    
    if (heuristicType_ == "manhattan") {
        return static_cast<float>(dx + dy);
    } else if (heuristicType_ == "euclidean") {
        return sqrtf(static_cast<float>(dx * dx + dy * dy));
    } else if (heuristicType_ == "octile" && eightDirectional_) {
        int maxDiff = (dx > dy) ? dx : dy;
        int minDiff = (dx < dy) ? dx : dy;
        return static_cast<float>(maxDiff) + 0.414f * static_cast<float>(minDiff);
    } else {
        return static_cast<float>(dx + dy);
    }
}

DStarKey DynamicAStarPathfinder::calculateKey(const Point& pos) const {
    float minVal = (nodes_.at(pos).g < nodes_.at(pos).rhs) ? nodes_.at(pos).g : nodes_.at(pos).rhs;
    float k1 = minVal + calculateHeuristic(startPos_, pos) + km_;
    float k2 = minVal;
    return DStarKey(k1, k2);
}

void DynamicAStarPathfinder::updateVertex(const Point& pos, const Grid& grid) {
    if (!grid.inBounds(pos)) return;
    
    // Ensure node exists
    if (nodes_.find(pos) == nodes_.end()) {
        nodes_[pos] = DStarNode(pos);
    }
    
    DStarNode& node = nodes_[pos];
    
    if (pos != goalPos_) {
        // Calculate rhs value
        node.rhs = std::numeric_limits<float>::infinity();
        
        auto neighbors = grid.getNeighbors(pos);
        for (const Point& neighbor : neighbors) {
            if (grid.isFree(neighbor)) {
                float cost = getCost(pos, neighbor, grid);
                if (nodes_.find(neighbor) != nodes_.end()) {
                    float candidateRhs = nodes_[neighbor].g + cost;
                    if (candidateRhs < node.rhs) {
                        node.rhs = candidateRhs;
                    }
                }
            }
        }
    }
    
    // Remove from queue if present
    if (inQueue_.count(pos)) {
        inQueue_.erase(pos);
        // Note: Can't easily remove from priority queue, so we check in computeShortestPath
    }
    
    // Add to queue if inconsistent
    float diff1 = node.g - node.rhs;
    if (diff1 > 0.001f || diff1 < -0.001f) {
        node.key = calculateKey(pos);
        openQueue_.push(std::make_pair(node.key, pos));
        inQueue_.insert(pos);
    }
}

void DynamicAStarPathfinder::computeShortestPath(const Grid& grid) {
    while (!openQueue_.empty()) {
        auto current = openQueue_.top();
        DStarKey topKey = current.first;
        Point pos = current.second;
        openQueue_.pop();
        
        // Skip if not in queue anymore (removed by updateVertex)
        if (!inQueue_.count(pos)) continue;
        
        inQueue_.erase(pos);
        
        // Ensure node exists
        if (nodes_.find(pos) == nodes_.end()) continue;
        
        DStarNode& node = nodes_[pos];
        
        // Check if key is still valid
        DStarKey currentKey = calculateKey(pos);
        if (topKey < currentKey) {
            // Key has increased, re-insert with new key
            node.key = currentKey;
            openQueue_.push(std::make_pair(currentKey, pos));
            inQueue_.insert(pos);
        } else if (node.g > node.rhs) {
            // Overconsistent case
            node.g = node.rhs;
            
            // Update neighbors
            auto neighbors = grid.getNeighbors(pos);
            for (const Point& neighbor : neighbors) {
                if (grid.inBounds(neighbor)) {
                    updateVertex(neighbor, grid);
                }
            }
        } else {
            // Underconsistent case
            node.g = std::numeric_limits<float>::infinity();
            
            // Update this vertex and neighbors
            updateVertex(pos, grid);
            auto neighbors = grid.getNeighbors(pos);
            for (const Point& neighbor : neighbors) {
                if (grid.inBounds(neighbor)) {
                    updateVertex(neighbor, grid);
                }
            }
        }
        
        // Check termination condition
        if (nodes_.find(startPos_) != nodes_.end()) {
            DStarNode& startNode = nodes_[startPos_];
            float diff = startNode.g - startNode.rhs;
            if ((diff > -0.001f && diff < 0.001f) && 
                (openQueue_.empty() || calculateKey(startPos_) < openQueue_.top().first)) {
                break;
            }
        }
    }
}

std::vector<Point> DynamicAStarPathfinder::extractPath(const Grid& grid) const {
    std::vector<Point> path;
    Point current = startPos_;
    
    // Follow gradient from start to goal
    while (current != goalPos_) {
        path.push_back(current);
        
        if (nodes_.find(current) == nodes_.end()) break;
        
        Point bestNext = current;
        float bestCost = std::numeric_limits<float>::infinity();
        
        auto neighbors = grid.getNeighbors(current);
        for (const Point& neighbor : neighbors) {
            if (grid.isFree(neighbor) && nodes_.find(neighbor) != nodes_.end()) {
                float cost = getCost(current, neighbor, grid) + nodes_.at(neighbor).g;
                if (cost < bestCost) {
                    bestCost = cost;
                    bestNext = neighbor;
                }
            }
        }
        
        if (bestNext == current) break; // No progress possible
        current = bestNext;
        
        // Prevent infinite loops
        if (path.size() > 1000) break;
    }
    
    if (current == goalPos_) {
        path.push_back(goalPos_);
    }
    
    return path;
}

float DynamicAStarPathfinder::getCost(const Point& from, const Point& to, const Grid& grid) const {
    if (!grid.isFree(to)) return std::numeric_limits<float>::infinity();
    
    float baseCost = grid.getCost(to);
    
    if (eightDirectional_) {
        Point diff = to - from;
        int absDx = (diff.x < 0) ? -diff.x : diff.x;
        int absDy = (diff.y < 0) ? -diff.y : diff.y;
        if (absDx + absDy == 2) {
            baseCost *= 1.414f; // sqrt(2) for diagonal
        }
    }
    
    return baseCost;
}

void DynamicAStarPathfinder::initialize(const Grid& grid, const Point& start, const Point& goal) {
    startPos_ = start;
    goalPos_ = goal;
    
    nodes_.clear();
    while (!openQueue_.empty()) openQueue_.pop();
    inQueue_.clear();
    
    // Initialize goal node
    nodes_[goalPos_] = DStarNode(goalPos_);
    nodes_[goalPos_].rhs = 0.0f;
    nodes_[goalPos_].key = calculateKey(goalPos_);
    
    openQueue_.push(std::make_pair(nodes_[goalPos_].key, goalPos_));
    inQueue_.insert(goalPos_);
    // Reset km when reinitializing
    km_ = 0.0f;
    
    std::cout << "D* Lite: Initialized from (" << start.x << "," << start.y 
              << ") to (" << goal.x << "," << goal.y << ")" << std::endl;
}
