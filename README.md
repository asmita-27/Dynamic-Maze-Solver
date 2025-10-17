# Dynamic Maze Solver

An interactive pathfinding visualization application featuring **A\*** and **Dynamic A\* (D\* Lite)** algorithms with real-time replanning, progressive difficulty levels, and intelligent loop detection.

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Algorithms](#algorithms)
- [Game Modes](#game-modes)
- [Controls](#controls)
- [Difficulty Levels](#difficulty-levels)
- [Requirements](#requirements)
- [Building](#building)
- [Usage](#usage)
- [Technical Details](#technical-details)
- [Project Structure](#project-structure)

## 🎯 Overview

Dynamic Maze Solver is an educational and interactive application that demonstrates advanced pathfinding algorithms in dynamic environments. Navigate through progressively challenging mazes using arrow keys while the application intelligently replans paths around obstacles in real-time.

### 🌟 Key Highlights

- **🎮 Arrow Key Navigation**: Intuitive directional movement with instant path recalculation
- **🔄 Automatic Replanning**: Path updates after every move using Dynamic A\*
- **🎚️ 5 Difficulty Levels**: Progressive challenge from 10×10 to 30×30 grids
- **🔀 Dual Modes**: Manual obstacle placement or automatic dynamic obstacles
- **🔁 Loop Detection**: Intelligent detection and manual override for stuck situations
- **✨ Professional UI**: Clean Arial font with centered, polished interface
- **🎯 Smart Obstacles**: Auto-generated obstacles target planned path points

## ✨ Features

### Core Gameplay Features

- ✅ **Arrow Key Movement** (↑↓←→): Direct agent control with automatic replanning
- ✅ **Real-time Path Replanning**: Path recalculates after every move
- ✅ **Mode Switching** (M key): Toggle between Manual and Auto modes during gameplay
- ✅ **Loop Detection System**: Tracks position history to detect getting stuck
- ✅ **Manual Override** (SPACEBAR): Take control in Auto mode when loops detected
- ✅ **5 Progressive Levels**: Increasing grid size and complexity
- ✅ **Quick Exit** (ESC/Q): Return to menu without closing application
- ✅ **Smart Obstacle Placement**: Targets path points 2+ steps ahead in Auto mode

### UI & Visualization

- **Professional Typography**: Anti-aliased Arial font throughout
- **Color-Coded Elements**:
  - 🟩 Green = Start position
  - 🟥 Red = Goal position
  - 🟦 Blue = Calculated path
  - 🟡 Yellow = Agent
  - ⬛ Black = Obstacles
- **Right-Side Information Panel**: Controls, instructions, and legend
- **Centered Layout**: All content properly aligned and balanced
- **Visual Feedback**: Mode indicators, override status, level progress

### Intelligent Systems

- **Position Tracking**: Monitors last 8 positions for loop detection
- **Revisit Counter**: Detects 3+ visits to same position
- **Automatic Warnings**: Console feedback with emoji indicators
- **Manual Intervention**: Enables click-to-toggle after loop detection
- **Reset Capability**: Clear loop detection when user takes control

## 🧮 Algorithms

### A\* (A-Star) Pathfinding

**Initial Path Calculation**
- Optimal pathfinding using heuristic search
- Manhattan distance heuristic
- Guaranteed shortest path in static environment

**Characteristics:**
- Complete and optimal
- Time Complexity: O(b^d) where b = branching factor, d = depth
- Space Complexity: O(b^d)

### Dynamic A\* (D\* Lite)

**Incremental Replanning**
- Efficiently updates paths when obstacles change
- Only recalculates affected portions
- Key-based priority system for efficient updates

**Advantages:**
- Faster replanning than full A\* recalculation
- Ideal for dynamic environments
- Maintains optimality while being efficient

**Implementation:**
- Plans from goal to start (backward search)
- Two-key priority system
- Local repair of paths

## 🎮 Game Modes

### Manual Mode 🟢

**Features:**
- Click cells to toggle obstacles
- Full control over maze layout
- Path replans after each move
- Ideal for exploring pathfinding behavior

**Best For:**
- Learning how algorithms respond to obstacles
- Testing specific scenarios
- Understanding path optimization

### Auto Mode 🔴

**Features:**
- Obstacles toggle automatically after each move
- 1-3 obstacles based on difficulty level
- Smart targeting of planned path
- Loop detection with manual override option

**Special Feature - Loop Detection:**
- Tracks last 8 agent positions
- Detects when stuck in loops (3+ revisits)
- Triggers after 2 loop detections
- **SPACEBAR** enables manual override
- Visual indicator: "MANUAL OVERRIDE ON"
- Console warnings with feedback

**Best For:**
- Demonstrating dynamic replanning
- Progressive challenge
- Testing algorithm robustness

## 🎛️ Controls

### Main Controls
| Key | Action |
|-----|--------|
| **↑↓←→** | Move agent (replans after each move) |
| **M** | Switch between Manual/Auto mode |
| **SPACEBAR** | Toggle manual override (Auto mode only) |
| **MOUSE** | Click cells to toggle obstacles (Manual mode) |
| **N** | Next level (after completion) |
| **R** | Reset to main menu |
| **ESC / Q** | Quit to menu |

### Menu Navigation
| Action | Method |
|--------|--------|
| Select mode | Click Manual or Auto button |
| Choose level | Click level 1-5 button |
| Select start | Click any free cell |
| Select goal | Click any free cell (not start) |

## 📊 Difficulty Levels

| Level | Grid Size | Obstacles | Challenge |
|-------|-----------|-----------|-----------|
| **1** | 10×10 | 1 per move | Easy - Learning |
| **2** | 15×15 | 1-2 per move | Medium - Practice |
| **3** | 20×20 | 2 per move | Hard - Challenging |
| **4** | 25×25 | 2-3 per move | Very Hard - Complex |
| **5** | 30×30 | 3 per move | Expert - Master |

**Progressive Difficulty:**
- Larger grids = longer paths
- More obstacles = more replanning
- Increased chance of loops in Auto mode
- Tests pathfinding efficiency at scale

## 💻 Requirements

### Software
- **OS**: Windows 10/11
- **Compiler**: MinGW-w64 GCC/G++ (C++14)
- **Graphics**: OpenGL 1.1+ support

### Hardware
- **RAM**: 2GB minimum
- **GPU**: OpenGL-compatible graphics
- **Display**: 1000×768 minimum resolution

### Dependencies
All standard Windows libraries:
- `opengl32.dll`
- `gdi32.dll`
- `user32.dll`
- `kernel32.dll`
- `winmm.dll`

## � Building

### Quick Build (Recommended)

```batch
build_simple.bat
```

This will:
- Clean previous builds
- Compile all source files with G++
- Link OpenGL and Windows libraries
- Generate `MazeSolver.exe`
- Launch the application

### Manual Build

```batch
g++ -std=c++14 -Wall -Wextra -O2 -Iinclude -mwindows ^
    -static-libgcc -static-libstdc++ ^
    src/main.cpp src/Config.cpp src/Grid.cpp ^
    src/Pathfinder.cpp src/AStarPathfinder.cpp ^
    src/DynamicAStarPathfinder.cpp src/Agent.cpp ^
    src/SimpleRenderer.cpp ^
    -lopengl32 -lgdi32 -luser32 -lkernel32 -lwinmm ^
    -o MazeSolver.exe
```

### Build Options

- `-O2`: Optimization level 2
- `-mwindows`: Windows GUI application
- `-static-libgcc -static-libstdc++`: Static linking for portability
- `-Iinclude`: Include directory for headers

## 🎮 Usage

### Starting the Application

1. **Run the executable:**
   ```batch
   MazeSolver.exe
   ```

2. **Main Menu appears** with:
   - Controls information (left panel)
   - Game information (right panel)
   - START button (center)

### Gameplay Flow

```
Main Menu → Select Mode → Select Level → Set Start → Set Goal → Play!
```

#### Step-by-Step:

1. **Click START** on main menu
2. **Choose Mode:**
   - Click **MANUAL** for obstacle control
   - Click **AUTO** for dynamic obstacles
3. **Select Difficulty:**
   - Click **LEVEL 1-5** based on skill
4. **Set Start Position:**
   - Click any cell (turns green)
5. **Set Goal Position:**
   - Click another cell (turns red)
6. **Navigate:**
   - Use **arrow keys** to move
   - Blue path shows optimal route
   - Yellow circle is your agent
7. **Reach Goal:**
   - Press **N** for next level
   - Press **R** to reset

### Tips for Success

**Manual Mode:**
- Start with few obstacles
- Observe how path recalculates
- Try blocking direct paths
- Watch alternate routes form

**Auto Mode:**
- Obstacles appear after each move
- If stuck in loop, press **SPACEBAR**
- Manual override lets you click obstacles
- Watch console for loop warnings

**Level Progression:**
- Complete all 5 levels
- Each level increases complexity
- Higher levels = more obstacles
- Test your pathfinding skills!

## 🏗️ Project Structure

```
Dynamic-Maze-Solver/
├── src/
│   ├── main.cpp                    # Main application & UI
│   ├── Config.cpp                  # Configuration management
│   ├── Grid.cpp                    # Grid and cell management
│   ├── Pathfinder.cpp              # Base pathfinder class
│   ├── AStarPathfinder.cpp         # A* implementation
│   ├── DynamicAStarPathfinder.cpp  # D* Lite implementation
│   ├── Agent.cpp                   # Agent movement logic
│   └── SimpleRenderer.cpp          # OpenGL rendering
├── include/
│   ├── Config.h
│   ├── Grid.h
│   ├── Point.h
│   ├── Pathfinder.h
│   ├── AStarPathfinder.h
│   ├── DynamicAStarPathfinder.h
│   ├── Agent.h
│   └── SimpleRenderer.h
├── build_simple.bat                # Build script
├── config.txt                      # Configuration file
└── README.md
```

### Key Components

#### Core Classes

**Grid** (`Grid.cpp/.h`)
- Manages maze grid structure
- Handles cell types (free, obstacle, start, goal)
- Validates positions and paths
- Provides neighbor queries

**Pathfinder** (`Pathfinder.cpp/.h`)
- Base class for pathfinding algorithms
- Defines common interface
- Handles path storage

**AStarPathfinder** (`AStarPathfinder.cpp/.h`)
- Implements A* algorithm
- Uses Manhattan heuristic
- Finds optimal paths efficiently

**DynamicAStarPathfinder** (`DynamicAStarPathfinder.cpp/.h`)
- Implements D* Lite algorithm
- Handles dynamic replanning
- Updates affected portions only
- More efficient for changing environments

**Agent** (`Agent.cpp/.h`)
- Controls agent movement
- Manages current position
- Stores and updates path
- Handles replanning triggers

**SimpleRenderer** (`SimpleRenderer.cpp/.h`)
- OpenGL-based rendering
- Draws grid, path, and agent
- Manages window and viewport
- Renders UI elements

#### UI System (`main.cpp`)

**Game States:**
- `MAIN_MENU`: Initial screen
- `MODE_SELECTION`: Choose Manual/Auto
- `LEVEL_SELECTION`: Choose difficulty
- `SELECTING_START`: Set start position
- `SELECTING_GOAL`: Set goal position
- `PLAYING`: Active gameplay
- `LEVEL_COMPLETE`: Goal reached

**Features:**
- Button system for menu navigation
- Mouse click handling
- Keyboard input processing
- OpenGL bitmap font rendering
- Loop detection system
- Manual override mechanism

## 🔬 Technical Details

### Pathfinding Implementation

**A* Algorithm:**
```
1. Initialize open list with start node
2. While open list not empty:
   a. Get node with lowest f = g + h
   b. If node is goal, reconstruct path
   c. For each neighbor:
      - Calculate g (cost from start)
      - Calculate h (heuristic to goal)
      - Add to open list if better path
3. Return path or failure
```

**D* Lite Algorithm:**
```
1. Initialize all nodes
2. Set goal node cost to 0
3. Plan initial path backward from goal
4. When obstacle changes:
   a. Update affected nodes
   b. Recompute only changed areas
   c. Update path incrementally
5. Agent follows updated path
```

### Loop Detection System

**Algorithm:**
```cpp
1. Track last 8 positions in circular buffer
2. On each move:
   a. Check if current position in history
   b. Count number of revisits
   c. If revisits >= 3: Loop detected
3. After 2 loop detections:
   a. Enable manual override flag
   b. Show visual indicator
   c. Allow spacebar toggle
   d. Enable click-to-toggle obstacles
```

**Data Structures:**
- `vector<Point> g_recentPositions` - Position history (size 8)
- `int g_loopDetectionCount` - Number of loops detected
- `bool g_allowManualToggleInAuto` - Override enabled flag

### Rendering System

**Font Rendering:**
- Uses `wglUseFontBitmaps` for OpenGL text
- Creates display lists for each character
- Arial font with anti-aliasing
- Size scales based on context

**Color Scheme:**
- Background: Dark blue-gray (0.1, 0.15, 0.2)
- Free cells: Light gray (0.95, 0.95, 0.95)
- Obstacles: Black (0.1, 0.1, 0.1)
- Start: Green (0.2, 1.0, 0.2)
- Goal: Red (1.0, 0.2, 0.2)
- Path: Blue (0.3, 0.6, 1.0)
- Agent: Yellow (1.0, 0.9, 0.0)

### Performance Optimization

- **Event-driven rendering**: Only redraws on changes
- **Efficient replanning**: D* Lite updates only affected nodes
- **Smart obstacle placement**: Targets path for maximum impact
- **Memory management**: Static allocation where possible
- **VSync enabled**: 30 FPS cap for smooth rendering

## 📝 Configuration

Edit `config.txt` to customize:

```ini
# Grid settings
grid_width=15
grid_height=15

# Movement
allow_diagonal=true

# Algorithm
heuristic=manhattan
algorithm=dynamic_astar

# Visualization
render_delay_ms=30
```

**Available Options:**
- `heuristic`: manhattan, euclidean, octile
- `algorithm`: astar, dynamic_astar
- `allow_diagonal`: true, false

## 🎓 Educational Value

### Learning Objectives

**Algorithm Understanding:**
- Visualize how A* explores the search space
- Understand heuristic-driven search
- See incremental replanning in action
- Compare static vs. dynamic algorithms

**Problem Solving:**
- Path optimization strategies
- Handling dynamic environments
- Loop detection and resolution
- Trade-offs between algorithms

**Programming Concepts:**
- Event-driven architecture
- State machine patterns
- OpenGL rendering
- User input handling

### Use Cases

- **University Projects**: AI/Algorithm courses
- **Research**: Pathfinding algorithm comparison
- **Teaching Tool**: Interactive demonstrations
- **Game Development**: Pathfinding prototyping

## 🐛 Troubleshooting

### Common Issues

**Application doesn't start:**
```batch
# Check MinGW installation
g++ --version

# Verify OpenGL support
# Run dxdiag and check Display tab
```

**Text not appearing:**
- Ensure Arial font is installed
- Check Windows font folder
- Application will fallback to system font

**Performance issues:**
- Reduce grid size in config.txt
- Disable VSync if needed
- Close background applications

**Loop detection too sensitive:**
- Increase position history size (line 68 in main.cpp)
- Adjust revisit threshold (line 83 in main.cpp)

## 🤝 Contributing

Contributions welcome! Areas for improvement:

- Additional pathfinding algorithms (Dijkstra, JPS)
- More heuristic options
- Save/load maze layouts
- Replay functionality
- Performance metrics display
- Additional themes/colors

## 📄 License

This project is created for educational purposes. Feel free to use and modify for learning and teaching.

## 👥 Authors

- **AI Lab Project** - Dynamic Maze Solver
- Course: 5th Semester AI Lab

## 🙏 Acknowledgments

- A* algorithm by Peter Hart, Nils Nilsson, and Bertram Raphael (1968)
- D* Lite algorithm by Sven Koenig and Maxim Likhachev (2002)
- OpenGL for graphics rendering
- MinGW for Windows compilation

---

**⭐ Star this repository if you find it helpful!**

**📧 Report issues or suggest features in the Issues tab**

---

**⭐ Star this repository if you find it helpful!**

**📧 Report issues or suggest features in the Issues tab**

