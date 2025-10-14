#include "Agent.h"
#include "AStarPathfinder.h"
#include "DynamicAStarPathfinder.h"
#include <iostream>
#include <windows.h>

Agent::Agent(const std::string& pathfinderType) 
    : position_(0, 0), start_(0, 0), goal_(0, 0), pathIndex_(0), 
      needsReplanning_(true), lastMoveTime_(0), moveInterval_(333), 
      manualMode_(true) {  // Start in manual mode
    
    setPathfinder(pathfinderType);
}

void Agent::setMovementSpeed(int stepsPerSecond) {
    if (stepsPerSecond < 1) stepsPerSecond = 1;
    if (stepsPerSecond > 10) stepsPerSecond = 10;
    
    moveInterval_ = 1000 / stepsPerSecond; // Convert to milliseconds
    
    std::cout << "Agent: Movement speed set to " << stepsPerSecond << " steps/second" << std::endl;
}

void Agent::setPathfinder(const std::string& pathfinderType) {
    if (pathfinderType == "AStar" || pathfinderType == "A*") {
        pathfinder_ = std::make_unique<AStarPathfinder>();
        std::cout << "Agent: Pathfinder set to A*" << std::endl;
    } else if (pathfinderType == "DynamicAStar" || pathfinderType == "Dynamic" || 
               pathfinderType == "DStar" || pathfinderType == "D*") {
        pathfinder_ = std::make_unique<DynamicAStarPathfinder>();
        std::cout << "Agent: Pathfinder set to Dynamic A* (D* Lite)" << std::endl;
    } else {
        // Default to A*
        pathfinder_ = std::make_unique<AStarPathfinder>();
        std::cout << "Agent: Unknown pathfinder type '" << pathfinderType << "', defaulting to A*" << std::endl;
    }
    
    // Configure pathfinder
    if (pathfinder_) {
        pathfinder_->setHeuristic("manhattan");
        pathfinder_->setMovementType(false); // Will be updated by grid
    }
    
    needsReplanning_ = true;
}

void Agent::planPath(const Grid& grid) {
    if (!pathfinder_) {
        std::cout << "Agent: No pathfinder available" << std::endl;
        return;
    }
    
    std::cout << "Agent: Planning path from (" << position_.x << "," << position_.y 
              << ") to (" << goal_.x << "," << goal_.y << ")" << std::endl;
    
    // Update pathfinder settings
    pathfinder_->setMovementType(grid.isEightDirectional());
    
    lastResult_ = pathfinder_->findPath(grid, position_, goal_);
    
    if (lastResult_.success) {
        currentPath_ = lastResult_.path;
        pathIndex_ = 0;
        needsReplanning_ = false;
        
        std::cout << "Agent: Path planned successfully - " << currentPath_.size() << " steps" << std::endl;
    } else {
        currentPath_.clear();
        pathIndex_ = 0;
        std::cout << "Agent: Path planning failed!" << std::endl;
    }
}

void Agent::update(Grid& grid) {
    // Check if replanning is needed
    if (needsReplanning(grid)) {
        // For Dynamic A*, try repair first if grid changed
        if (getPathfinderName().find("Dynamic") != std::string::npos && 
            grid.hasChanges() && !currentPath_.empty()) {
            
            std::cout << "Agent: Attempting path repair..." << std::endl;
            lastResult_ = pathfinder_->repairPath(grid, position_);
            
            if (lastResult_.success) {
                currentPath_ = lastResult_.path;
                pathIndex_ = 0;
                needsReplanning_ = false;
                std::cout << "Agent: Path repaired successfully" << std::endl;
            } else {
                std::cout << "Agent: Path repair failed, doing full replan" << std::endl;
                planPath(grid);
            }
        } else {
            planPath(grid);
        }
        
        grid.clearChanges();
    }
    
    // Only auto-move if not in manual mode
    if (!manualMode_) {
        moveToNext();
    }
}

void Agent::reset() {
    position_ = start_;
    pathIndex_ = 0;
    currentPath_.clear();
    needsReplanning_ = true;
    lastMoveTime_ = 0;
    
    if (pathfinder_) {
        pathfinder_->reset();
    }
    
    std::cout << "Agent: Reset to start position (" << start_.x << "," << start_.y << ")" << std::endl;
}

void Agent::stepForward(Grid& grid) {
    // First, check if we need to replan
    if (needsReplanning(grid)) {
        // For Dynamic A*, try repair first if grid changed
        if (getPathfinderName().find("Dynamic") != std::string::npos && 
            grid.hasChanges() && !currentPath_.empty()) {
            
            std::cout << "Agent: Attempting path repair after move..." << std::endl;
            lastResult_ = pathfinder_->repairPath(grid, position_);
            
            if (lastResult_.success) {
                currentPath_ = lastResult_.path;
                pathIndex_ = 0;
                needsReplanning_ = false;
                std::cout << "Agent: Path repaired successfully" << std::endl;
            } else {
                std::cout << "Agent: Path repair failed, doing full replan" << std::endl;
                planPath(grid);
            }
        } else {
            planPath(grid);
        }
        
        grid.clearChanges();
    }
    
    // Now try to move one step
    if (currentPath_.empty() || pathIndex_ >= currentPath_.size()) {
        std::cout << "Agent: No valid path to move" << std::endl;
        return;
    }
    
    if (pathIndex_ + 1 < currentPath_.size()) {
        pathIndex_++;
        position_ = currentPath_[pathIndex_];
        
        std::cout << "Agent: Moved to (" << position_.x << "," << position_.y << ")";
        
        if (hasReachedGoal()) {
            std::cout << " - GOAL REACHED!" << std::endl;
        } else {
            int remaining = currentPath_.size() - pathIndex_ - 1;
            std::cout << " [" << remaining << " steps remaining]" << std::endl;
            
            // Immediately replan after each move to show updated path
            needsReplanning_ = true;
        }
    } else {
        std::cout << "Agent: Already at end of path" << std::endl;
    }
}

std::string Agent::getPathfinderName() const {
    return pathfinder_ ? pathfinder_->getName() : "None";
}

int Agent::getTotalNodesExpanded() const {
    return pathfinder_ ? pathfinder_->getTotalNodesExpanded() : 0;
}

std::chrono::milliseconds Agent::getTotalPlanningTime() const {
    return pathfinder_ ? pathfinder_->getTotalPlanningTime() : std::chrono::milliseconds(0);
}

bool Agent::needsReplanning(const Grid& grid) const {
    // Force replanning if flagged
    if (needsReplanning_) {
        return true;
    }
    
    // Replan if grid changed
    if (grid.hasChanges()) {
        std::cout << "Agent: Grid changed - replanning needed" << std::endl;
        return true;
    }
    
    // Replan if current path is invalid
    if (!pathfinder_->isPathValid(currentPath_, grid)) {
        std::cout << "Agent: Current path blocked - replanning needed" << std::endl;
        return true;
    }
    
    // Replan if next step is blocked
    if (pathIndex_ + 1 < currentPath_.size()) {
        Point nextStep = currentPath_[pathIndex_ + 1];
        if (!grid.isFree(nextStep)) {
            std::cout << "Agent: Next step blocked - immediate replanning needed" << std::endl;
            return true;
        }
    }
    
    return false;
}

void Agent::moveToNext() {
    if (currentPath_.empty() || pathIndex_ >= currentPath_.size()) {
        return; // No valid path or reached end
    }
    
    DWORD currentTime = GetTickCount();
    if (currentTime - lastMoveTime_ < moveInterval_) {
        return; // Not time to move yet
    }
    
    if (pathIndex_ + 1 < currentPath_.size()) {
        pathIndex_++;
        position_ = currentPath_[pathIndex_];
        lastMoveTime_ = currentTime;
        
        std::cout << "Agent: Moved to (" << position_.x << "," << position_.y << ")";
        
        if (hasReachedGoal()) {
            std::cout << " - GOAL REACHED!" << std::endl;
        } else {
            int remaining = currentPath_.size() - pathIndex_ - 1;
            std::cout << " [" << remaining << " steps remaining]" << std::endl;
        }
    }
}
