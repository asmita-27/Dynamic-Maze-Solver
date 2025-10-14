#pragma once
#include "Point.h"
#include <string>

class Config {
public:
    Config();
    void load(const std::string& filename = "config.txt");
    
    // Getters
    int getGridWidth() const { return gridWidth_; }
    int getGridHeight() const { return gridHeight_; }
    Point getStartPosition() const { return startPosition_; }
    Point getGoalPosition() const { return goalPosition_; }
    float getObstacleDensity() const { return obstacleDensity_; }
    bool isEightDirectional() const { return eightDirectional_; }
    int getSimulationSpeed() const { return simulationSpeed_; }
    std::string getAlgorithm() const { return algorithm_; }
    float getRandomEventProbability() const { return randomEventProbability_; }
    
    bool isValid() const { return valid_; }

private:
    void setDefaults();
    void validateAndFix();
    
    int gridWidth_;
    int gridHeight_;
    Point startPosition_;
    Point goalPosition_;
    float obstacleDensity_;
    bool eightDirectional_;
    int simulationSpeed_;
    std::string algorithm_;
    float randomEventProbability_;
    bool valid_;
};
