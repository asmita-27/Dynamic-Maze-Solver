#include "Config.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>

Config::Config() : valid_(false) {
    setDefaults();
}

void Config::setDefaults() {
    gridWidth_ = 25;
    gridHeight_ = 15;
    startPosition_ = Point(1, 1);
    goalPosition_ = Point(23, 13);
    obstacleDensity_ = 0.18f;
    eightDirectional_ = false;
    simulationSpeed_ = 3;
    algorithm_ = "AStar";
    randomEventProbability_ = 0.02f;
    valid_ = true;
}

void Config::load(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cout << "Config file not found: " << filename << ". Using defaults." << std::endl;
        validateAndFix();
        return;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        size_t pos = line.find('=');
        if (pos == std::string::npos) continue;
        
        std::string key = line.substr(0, pos);
        std::string value = line.substr(pos + 1);
        
        // Trim whitespace
        key.erase(0, key.find_first_not_of(" \t"));
        key.erase(key.find_last_not_of(" \t") + 1);
        value.erase(0, value.find_first_not_of(" \t"));
        value.erase(value.find_last_not_of(" \t") + 1);
        
        try {
            if (key == "grid_width") {
                gridWidth_ = std::stoi(value);
            } else if (key == "grid_height") {
                gridHeight_ = std::stoi(value);
            } else if (key == "obstacle_density") {
                obstacleDensity_ = std::stof(value);
            } else if (key == "start_x") {
                startPosition_.x = std::stoi(value);
            } else if (key == "start_y") {
                startPosition_.y = std::stoi(value);
            } else if (key == "goal_x") {
                goalPosition_.x = std::stoi(value);
            } else if (key == "goal_y") {
                goalPosition_.y = std::stoi(value);
            } else if (key == "eight_directional") {
                eightDirectional_ = (value == "true" || value == "1");
            } else if (key == "simulation_speed") {
                simulationSpeed_ = std::stoi(value);
            } else if (key == "algorithm") {
                algorithm_ = value;
            } else if (key == "random_event_probability") {
                randomEventProbability_ = std::stof(value);
            }
        } catch (const std::exception& e) {
            std::cout << "Error parsing config value for " << key << ": " << e.what() << std::endl;
        }
    }
    
    validateAndFix();
    
    std::cout << "Configuration loaded:" << std::endl;
    std::cout << "  Grid: " << gridWidth_ << "x" << gridHeight_ << std::endl;
    std::cout << "  Start: (" << startPosition_.x << "," << startPosition_.y << ")" << std::endl;
    std::cout << "  Goal: (" << goalPosition_.x << "," << goalPosition_.y << ")" << std::endl;
    std::cout << "  Movement: " << (eightDirectional_ ? "8" : "4") << "-directional" << std::endl;
    std::cout << "  Obstacle density: " << (obstacleDensity_ * 100) << "%" << std::endl;
    std::cout << "  Speed: " << simulationSpeed_ << " steps/second" << std::endl;
    std::cout << "  Algorithm: " << algorithm_ << std::endl;
    std::cout << "  Random event probability: " << (randomEventProbability_ * 100) << "%" << std::endl;
}

void Config::validateAndFix() {
    // Clamp values to safe ranges
    gridWidth_ = std::max(10, std::min(50, gridWidth_));
    gridHeight_ = std::max(10, std::min(50, gridHeight_));
    obstacleDensity_ = std::max(0.0f, std::min(0.4f, obstacleDensity_));
    simulationSpeed_ = std::max(1, std::min(10, simulationSpeed_));
    randomEventProbability_ = std::max(0.0f, std::min(0.1f, randomEventProbability_));
    
    // Fix positions to be within bounds
    startPosition_.x = std::max(0, std::min(gridWidth_ - 1, startPosition_.x));
    startPosition_.y = std::max(0, std::min(gridHeight_ - 1, startPosition_.y));
    goalPosition_.x = std::max(0, std::min(gridWidth_ - 1, goalPosition_.x));
    goalPosition_.y = std::max(0, std::min(gridHeight_ - 1, goalPosition_.y));
    
    // Ensure start and goal are different
    if (startPosition_ == goalPosition_) {
        if (goalPosition_.x < gridWidth_ - 1) {
            goalPosition_.x++;
        } else if (goalPosition_.y < gridHeight_ - 1) {
            goalPosition_.y++;
        } else {
            goalPosition_.x--;
        }
    }
    
    valid_ = true;
}
