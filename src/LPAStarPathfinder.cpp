#include "LPAStarPathfinder.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <chrono>

LPAStarPathfinder::LPAStarPathfinder() 
    : heuristicType_("manhattan"), eightDirectional_(false), 
      totalNodesExpanded_(0), totalPlanningTime_(0), initialized_(false) {
}

void LPAStarPathfinder::reset() {
    nodes_.clear();
    while (!openQueue_.empty()) openQueue_.pop();
    inQueue_.clear();
    totalNodesExpanded_ = 0;
    totalPlanningTime_ = std::chrono::milliseconds(0);
    initialized_ = false;
}

void LPAStarPathfinder::setHeuristic(const std::string& heuristicType) {
    heuristicType_ = heuristicType;
}

void LPAStarPathfinder::setMovementType(bool eightDirectional) {
    eightDirectional_ = eightDirectional;
}

float LPAStarPathfinder::calculateHeuristic(const Point& from, const Point& to) const {
    int dx = std::abs(from.x - to.x);
    int dy = std::abs(from.y - to.y);
    
    if (heuristicType_ == "euclidean") {
        return std::sqrt(static_cast<float>(dx * dx + dy * dy));
    } else if (heuristicType_ == "diagonal") {
        float D = 1.0f;
        float D2 = 1.414f;
        return D * (dx + dy) + (D2 - 2 * D) * std::min(dx, dy);
    } else { // manhattan
        return static_cast<float>(dx + dy);
    }
}

LPAKey LPAStarPathfinder::calculateKey(const Point& pos) const {
    auto it = nodes_.find(pos);
    if (it == nodes_.end()) {
        return LPAKey(1000000.0f, 1000000.0f);
    }
    
    float minVal = std::min(it->second.g, it->second.rhs);
    float h = calculateHeuristic(pos, startPos_);
    return LPAKey(minVal + h, minVal);
}

float LPAStarPathfinder::getCost(const Point& from, const Point& to, const Grid& grid) const {
    if (!grid.isFree(to)) {
        return 1000000.0f;
    }
    
    int dx = std::abs(to.x - from.x);
    int dy = std::abs(to.y - from.y);
    
    // If this is a diagonal move (dx==1 && dy==1)
    if (dx + dy == 2) {
        // If diagonal movement is disabled, treat diagonal as impassable
        if (!eightDirectional_) {
            return 1000000.0f;
        }
        return 1.414f;
    }

    return 1.0f;
}

void LPAStarPathfinder::updateVertex(const Point& pos, const Grid& grid) {
    auto& node = nodes_[pos];
    node.position = pos;
    
    if (!(pos == goalPos_)) {
        float minRhs = 1000000.0f;
        
        std::vector<Point> neighbors;
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                if (dx == 0 && dy == 0) continue;
                if (!eightDirectional_ && dx != 0 && dy != 0) continue;
                
                Point neighbor(pos.x + dx, pos.y + dy);
                if (grid.inBounds(neighbor)) {
                    neighbors.push_back(neighbor);
                }
            }
        }
        
        for (const auto& succ : neighbors) {
            auto it = nodes_.find(succ);
            if (it != nodes_.end()) {
                float cost = getCost(pos, succ, grid);
                float candidateRhs = it->second.g + cost;
                if (candidateRhs < minRhs) {
                    minRhs = candidateRhs;
                }
            }
        }
        
        node.rhs = minRhs;
    }
    
    if (inQueue_.count(pos)) {
        inQueue_.erase(pos);
    }
    
    if (node.g != node.rhs) {
        node.key = calculateKey(pos);
        openQueue_.push({node.key, pos});
        inQueue_.insert(pos);
    }
}

void LPAStarPathfinder::computeShortestPath(const Grid& grid) {
    while (!openQueue_.empty()) {
        auto top = openQueue_.top();
        LPAKey topKey = top.first;
        Point topPos = top.second;
        openQueue_.pop();
        inQueue_.erase(topPos);
        
        LPAKey currentKey = calculateKey(startPos_);
        auto& startNode = nodes_[startPos_];
        
        if (topKey < currentKey || startNode.rhs != startNode.g) {
            totalNodesExpanded_++;
            
            auto& node = nodes_[topPos];
            
            if (node.g > node.rhs) {
                node.g = node.rhs;
                
                std::vector<Point> predecessors;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        if (dx == 0 && dy == 0) continue;
                        if (!eightDirectional_ && dx != 0 && dy != 0) continue;
                        
                        Point pred(topPos.x + dx, topPos.y + dy);
                        if (grid.inBounds(pred)) {
                            predecessors.push_back(pred);
                        }
                    }
                }
                
                for (const auto& pred : predecessors) {
                    updateVertex(pred, grid);
                }
            } else {
                node.g = 1000000.0f;
                updateVertex(topPos, grid);
                
                std::vector<Point> predecessors;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        if (dx == 0 && dy == 0) continue;
                        if (!eightDirectional_ && dx != 0 && dy != 0) continue;
                        
                        Point pred(topPos.x + dx, topPos.y + dy);
                        if (grid.inBounds(pred)) {
                            predecessors.push_back(pred);
                        }
                    }
                }
                
                for (const auto& pred : predecessors) {
                    updateVertex(pred, grid);
                }
            }
        } else {
            break;
        }
    }
}

void LPAStarPathfinder::initialize(const Grid& grid, const Point& start, const Point& goal) {
    reset();
    
    startPos_ = start;
    goalPos_ = goal;
    
    auto& goalNode = nodes_[goalPos_];
    goalNode.position = goalPos_;
    goalNode.rhs = 0.0f;
    goalNode.g = 1000000.0f;
    
    goalNode.key = calculateKey(goalPos_);
    openQueue_.push({goalNode.key, goalPos_});
    inQueue_.insert(goalPos_);
    
    initialized_ = true;
}

std::vector<Point> LPAStarPathfinder::extractPath(const Grid& grid) const {
    std::vector<Point> path;
    
    auto it = nodes_.find(startPos_);
    if (it == nodes_.end() || it->second.g >= 1000000.0f) {
        return path;
    }
    
    Point current = startPos_;
    path.push_back(current);
    
    while (!(current == goalPos_)) {
        float bestCost = 1000000.0f;
        Point bestNext = current;
        
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                if (dx == 0 && dy == 0) continue;
                if (!eightDirectional_ && dx != 0 && dy != 0) continue;
                
                Point neighbor(current.x + dx, current.y + dy);
                if (!grid.inBounds(neighbor)) continue;
                if (!grid.isFree(neighbor)) continue;
                auto neighborIt = nodes_.find(neighbor);
                if (neighborIt != nodes_.end()) {
                    float edgeCost = getCost(current, neighbor, grid);
                    float totalCost = neighborIt->second.g + edgeCost;
                    
                    if (totalCost < bestCost) {
                        bestCost = totalCost;
                        bestNext = neighbor;
                    }
                }
            }
        }
        
        if (bestNext == current) {
            break;
        }
        
        current = bestNext;
        path.push_back(current);
        
        if (path.size() > 10000) {
            break;
        }
    }
    
    return path;
}

PathfindingResult LPAStarPathfinder::findPath(const Grid& grid, const Point& start, const Point& goal) {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    PathfindingResult result;
    result.success = false;
    
    if (!grid.inBounds(start) || !grid.inBounds(goal)) {
        return result;
    }
    
    if (!grid.isFree(start) || !grid.isFree(goal)) {
        return result;
    }
    
    initialize(grid, start, goal);
    computeShortestPath(grid);
    
    auto& startNode = nodes_[startPos_];
    if (startNode.g < 1000000.0f) {
        result.path = extractPath(grid);
        result.success = !result.path.empty();
        
        if (result.success) {
            result.pathCost = startNode.g;
        }
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    result.planningTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    result.nodesExpanded = totalNodesExpanded_;
    
    totalPlanningTime_ += result.planningTime;
    
    return result;
}

PathfindingResult LPAStarPathfinder::repairPath(const Grid& grid, const Point& currentPos) {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    PathfindingResult result;
    result.success = false;
    
    if (!initialized_) {
        return findPath(grid, currentPos, goalPos_);
    }
    
    startPos_ = currentPos;
    
    int expandedBefore = totalNodesExpanded_;
    computeShortestPath(grid);
    
    auto& startNode = nodes_[startPos_];
    if (startNode.g < 1000000.0f) {
        result.path = extractPath(grid);
        result.success = !result.path.empty();
        
        if (result.success) {
            result.pathCost = startNode.g;
        }
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    result.planningTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    result.nodesExpanded = totalNodesExpanded_ - expandedBefore;
    
    totalPlanningTime_ += result.planningTime;
    
    return result;
}

void LPAStarPathfinder::updateGrid(const Grid& grid, const std::vector<Point>& changedCells) {
    if (!initialized_) return;
    
    for (const auto& cell : changedCells) {
        updateVertex(cell, grid);
        
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                if (dx == 0 && dy == 0) continue;
                if (!eightDirectional_ && dx != 0 && dy != 0) continue;
                
                Point neighbor(cell.x + dx, cell.y + dy);
                if (grid.inBounds(neighbor)) {
                    updateVertex(neighbor, grid);
                }
            }
        }
    }
}
