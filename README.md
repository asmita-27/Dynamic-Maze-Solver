# Dynamic Maze Solver

A real-time pathfinding visualization tool implementing **A\*** and **Dynamic A\* (D\* Lite)** algorithms with interactive obstacle editing and automatic replanning capabilities.

![Project Status](https://img.shields.io/badge/status-active-success.svg)
![Build](https://img.shields.io/badge/build-passing-brightgreen.svg)
![License](https://img.shields.io/badge/license-MIT-blue.svg)

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Algorithms Implemented](#algorithms-implemented)
- [Requirements](#requirements)
- [Installation](#installation)
- [Building the Project](#building-the-project)
- [Usage](#usage)
- [Controls](#controls)
- [Configuration](#configuration)
- [Project Structure](#project-structure)
- [Technical Details](#technical-details)
- [Screenshots](#screenshots)
- [Contributing](#contributing)
- [License](#license)

## 🎯 Overview

This project demonstrates advanced pathfinding algorithms in a dynamic environment where obstacles can change in real-time. The application visualizes how different algorithms handle path planning and replanning, making it an excellent educational tool for understanding AI search algorithms.

### Key Highlights

- **Real-time Visualization**: Watch algorithms find and update paths in real-time
- **Interactive Environment**: Toggle obstacles and see immediate replanning
- **Manual/Auto Modes**: Step through paths manually or watch automatic navigation
- **Dynamic Events**: Scheduled door events and random obstacle changes
- **Performance Metrics**: Track nodes expanded, planning time, and path costs

## ✨ Features

### Core Features

- ✅ **A\* Pathfinding**: Classic optimal pathfinding algorithm
- ✅ **Dynamic A\* (D\* Lite)**: Efficient incremental replanning algorithm
- ✅ **Manual Movement Mode**: Step-by-step path traversal with automatic replanning
- ✅ **Automatic Movement Mode**: Watch the agent navigate autonomously
- ✅ **Real-time Obstacle Editing**: Click to add/remove obstacles during execution
- ✅ **Multiple Heuristics**: Manhattan, Euclidean, and Octile distance options
- ✅ **4/8-Directional Movement**: Toggle between movement types
- ✅ **Scheduled Events**: Timed door opening/closing
- ✅ **Random Events**: Probabilistic obstacle appearance/disappearance
- ✅ **Path Validation**: Automatic checking for blocked paths
- ✅ **Performance Statistics**: Detailed metrics for algorithm comparison

### Visualization Features

- OpenGL-based rendering
- Color-coded cell types (start, goal, obstacles, path)
- Smooth visual updates
- Grid-based layout

## 🧮 Algorithms Implemented

### 1. A\* (A-Star)

A widely-used pathfinding algorithm that finds the optimal path using:
- **Heuristic Function**: Estimates distance to goal
- **Cost Function**: Actual cost from start
- **Priority Queue**: Explores most promising nodes first

**Use Case**: Best for static environments where obstacles don't change

### 2. Dynamic A\* (D\* Lite)

An incremental heuristic search algorithm designed for dynamic environments:
- **Efficient Replanning**: Only updates affected portions of the path
- **Key-based Priority**: Uses two-key priority system
- **Backward Search**: Plans from goal to start for efficient updates

**Use Case**: Optimal for environments with changing obstacles

## 💻 Requirements

### Software Requirements

- **Operating System**: Windows 10 or later
- **Compiler**: MinGW-w64 with GCC/G++ (C++14 or later)
- **Graphics**: OpenGL support (OpenGL 1.1 or higher)
- **Build Tool**: CMake 3.10+ (optional) or direct compilation

### Hardware Requirements

- **RAM**: 2GB minimum
- **Graphics**: GPU with OpenGL support
- **Processor**: Any modern CPU (Intel/AMD)

### Dependencies

- OpenGL32
- GDI32
- User32
- Kernel32
- WinMM

*Note: All dependencies are standard Windows libraries*

## 🚀 Installation

### 1. Clone the Repository

```bash
git clone https://github.com/yourusername/dynamic-maze-solver.git
cd dynamic-maze-solver
```

### 2. Install MinGW (if not already installed)

Download and install MinGW-w64 from:
- [MinGW-w64 Downloads](https://www.mingw-w64.org/downloads/)
- Or use [MSYS2](https://www.msys2.org/)

Add MinGW `bin` directory to your PATH.

### 3. Verify Installation

```bash
g++ --version
```

## 🔨 Building the Project

### Method 1: Using Build Script (Recommended)

Simply run the provided batch script:

```bash
build_simple.bat
```

This will:
- Compile all source files
- Link required libraries
- Create `MazeSolver.exe`
- Automatically launch the application

### Method 2: Using CMake

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

### Method 3: Manual Compilation

```bash
g++ -std=c++14 -Wall -Wextra -O2 -Iinclude -mwindows ^
    -static-libgcc -static-libstdc++ ^
    src/main.cpp src/Config.cpp src/Grid.cpp ^
    src/Pathfinder.cpp src/AStarPathfinder.cpp ^
    src/DynamicAStarPathfinder.cpp src/Agent.cpp ^
    src/SimpleRenderer.cpp ^
    -lopengl32 -lgdi32 -luser32 -lkernel32 -lwinmm ^
    -o MazeSolver.exe
```

## 🎮 Usage

### Running the Application

```bash
MazeSolver.exe
```

### Getting Started

1. **Initial Setup**: The application starts with:
   - A randomly generated maze
   - Agent at the start position (green cell)
   - Goal at the end position (red cell)
   - Initial path calculated and displayed

2. **Manual Mode** (Default):
   - Path is displayed but agent doesn't move
   - Press **Enter** or **Right Arrow** to move one step
   - Path automatically replans after each move

3. **Auto Mode**:
   - Press **M** to toggle to automatic mode
   - Agent moves automatically along the path
   - Watch real-time navigation

## ⌨️ Controls

### Movement Controls

| Key | Action |
|-----|--------|
| **Enter** / **→** | Move one step forward (Manual Mode) |
| **M** | Toggle Manual/Auto movement mode |

### Pathfinding Controls

| Key | Action |
|-----|--------|
| **Space** | Force replanning |
| **A** | Switch to A\* algorithm |
| **D** | Switch to Dynamic A\* (D\* Lite) |

### Environment Controls

| Key | Action |
|-----|--------|
| **Left Click** | Toggle obstacles (add/remove) |
| **C** | Clear all obstacles |
| **R** | Reset simulation to start |

### Application Controls

| Key | Action |
|-----|--------|
| **ESC** | Exit application |

## ⚙️ Configuration

Edit `config.txt` to customize the simulation:

```ini
# Grid dimensions
grid_width=40
grid_height=30

# Start and goal positions (x,y)
start_x=2
start_y=2
goal_x=37
goal_y=27

# Algorithm: AStar or DynamicAStar
algorithm=DynamicAStar

# Movement: true for 8-directional, false for 4-directional
eight_directional=false

# Obstacle density (0.0 to 1.0)
obstacle_density=0.25

# Agent speed (steps per second: 1-10)
simulation_speed=3

# Random events probability (0.0 to 1.0)
random_event_probability=0.0
```

## 📁 Project Structure

```
cp/
├── include/              # Header files
│   ├── Agent.h          # Agent controller
│   ├── AStarPathfinder.h
│   ├── Config.h         # Configuration management
│   ├── DynamicAStarPathfinder.h
│   ├── Grid.h           # Grid and cell management
│   ├── Pathfinder.h     # Base pathfinder interface
│   ├── Point.h          # Point structure with hash
│   └── SimpleRenderer.h # OpenGL renderer
├── src/                 # Source files
│   ├── Agent.cpp
│   ├── AStarPathfinder.cpp
│   ├── Config.cpp
│   ├── DynamicAStarPathfinder.cpp
│   ├── Grid.cpp
│   ├── main.cpp         # Application entry point
│   ├── Pathfinder.cpp
│   └── SimpleRenderer.cpp
├── build/               # Build artifacts
├── config.txt           # Configuration file
├── build_simple.bat     # Build script
├── CMakeLists.txt       # CMake configuration
└── README.md            # This file
```

## 🔧 Technical Details

### Architecture

The project follows object-oriented design principles:

- **Pathfinder Interface**: Abstract base class for pathfinding algorithms
- **Grid Manager**: Handles cell states, obstacles, and events
- **Agent Controller**: Manages agent position and navigation
- **Renderer**: OpenGL-based visualization layer
- **Config Manager**: Centralized configuration handling

### Data Structures

- **Priority Queue**: For A\* open list management
- **Unordered Map**: Fast node lookups with custom Point hash
- **Unordered Set**: Closed list and queue membership tracking
- **Vector**: Path storage and neighbor lists

### Performance Optimizations

- Efficient hash function for Point objects
- MinGW-compatible floating-point comparisons
- Incremental replanning with D\* Lite
- Static library linking for portability

### Heuristic Functions

1. **Manhattan Distance**: `|x1-x2| + |y1-y2|`
2. **Euclidean Distance**: `√((x1-x2)² + (y1-y2)²)`
3. **Octile Distance**: `max(|x1-x2|, |y1-y2|) + 0.414 * min(|x1-x2|, |y1-y2|)`

## 📸 Screenshots

*Add screenshots of your application here showing:*
- Initial path visualization
- Obstacle editing
- Path replanning
- Algorithm comparison
- Statistics display

## 🤝 Contributing

Contributions are welcome! Here's how you can help:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

### Areas for Improvement

- Additional pathfinding algorithms (Dijkstra, Jump Point Search)
- GUI for configuration
- Path smoothing
- Multiple agents
- Obstacle types with different costs
- Better visualization effects

## 📝 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 👥 Authors

- **Your Name** - *Initial work*

## 🙏 Acknowledgments

- A\* algorithm by Peter Hart, Nils Nilsson, and Bertram Raphael (1968)
- D\* Lite algorithm by Sven Koenig and Maxim Likhachev (2002)
- OpenGL community for graphics documentation
- MinGW-w64 project for Windows compilation tools

## 📚 References

- [A\* Pathfinding for Beginners](http://www.policyalmanac.org/games/aStarTutorial.htm)
- [D\* Lite Paper](http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf)
- [Introduction to A\*](https://www.redblobgames.com/pathfinding/a-star/introduction.html)
- [Pathfinding Algorithms](https://en.wikipedia.org/wiki/Pathfinding)

## 📧 Contact

For questions or support, please open an issue on GitHub.

---

**Note**: This project was developed as part of AI Lab coursework, demonstrating practical applications of search algorithms in dynamic environments.

Made with ❤️ and C++
