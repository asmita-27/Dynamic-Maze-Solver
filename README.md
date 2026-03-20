# Dynamic Maze Solver

An interactive pathfinding visualization application featuring **A\***, **Dynamic A\* (D\* Lite)**, and **LPA\*** algorithms with **real-time algorithm comparison**, intelligent auto-selection, multi-path visualization, and progressive difficulty levels.

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Algorithms](#algorithms)
- [Real-Time Comparator](#real-time-comparator)
- [Game Modes](#game-modes)
- [Controls](#controls)
- [Difficulty Levels](#difficulty-levels)
- [Requirements](#requirements)
- [Building](#building)
- [Usage](#usage)
- [Technical Details](#technical-details)
- [Project Structure](#project-structure)

## 🎯 Overview

Dynamic Maze Solver is an educational and interactive application that demonstrates advanced pathfinding algorithms in dynamic environments. Navigate through progressively challenging mazes while the application **compares three algorithms simultaneously**, showing real-time performance metrics and visualizing all paths with distinct colors.

### 🌟 Key Highlights

- **🔬 Real-Time Algorithm Comparison**: Run A\*, D\* Lite, and LPA\* simultaneously
- **🎨 Multi-Path Visualization**: See all algorithm paths with color coding (Cyan, Magenta, Green)
- **🤖 Intelligent Auto-Selection**: System picks optimal algorithm with tie-breaker logic
- **📊 Performance Metrics HUD**: Live timing, nodes expanded, path cost for each algorithm
- **🎮 Interactive Navigation**: Step-by-step movement with instant path recalculation
- **🔄 Automatic Replanning**: Path updates after every move using incremental algorithms
- **🎚️ 5 Difficulty Levels**: Progressive challenge from 10×10 to 30×30 grids
- **🔀 Dual Modes**: Manual obstacle placement or automatic dynamic obstacles
- **🔁 Loop Detection**: Intelligent detection and manual override for stuck situations
- **✨ Professional UI**: Clean interface with comprehensive metrics display

## ✨ Features

### Algorithm Comparison Features ⭐NEW

- ✅ **Three Pathfinding Algorithms**: A\*, Dynamic A\* (D\* Lite), and LPA\*
- ✅ **Simultaneous Execution**: All algorithms run in parallel for comparison
- ✅ **Real-Time Performance Metrics**: Display timing, nodes expanded, path cost
- ✅ **Visual Path Differentiation**: 
  - Cyan path = A\*
  - Magenta path = Dynamic A\*
  - Green path = LPA\*
  - Dark Blue = Chosen/Active path
- ✅ **Intelligent Algorithm Selection**: Automatic optimal algorithm choice
- ✅ **Tie-Breaker Logic**: Prefers dynamic algorithms when costs equal
- ✅ **OPT/CHOSEN Labels**: Shows which algorithms found optimal path and which is active
- ✅ **Comparator Toggle** (T key): Enable/disable comparison mode
- ✅ **Updated Legend**: Shows all path colors and their meanings

### Core Gameplay Features

- ✅ **Manual Step Control** (ENTER key): Step-by-step pathfinding demonstration
- ✅ **Real-time Path Replanning**: Path recalculates after every move
- ✅ **Mode Switching** (M key): Toggle between Manual and Auto modes during gameplay
- ✅ **Loop Detection System**: Tracks position history to detect getting stuck
- ✅ **Manual Override** (SPACEBAR): Take control in Auto mode when loops detected
- ✅ **5 Progressive Levels**: Increasing grid size and complexity
- ✅ **Quick Exit** (ESC/Q): Return to menu without closing application
- ✅ **Smart Obstacle Placement**: Targets path points 2+ steps ahead in Auto mode

### UI & Visualization

- **Professional Typography**: Anti-aliased font throughout
- **Color-Coded Elements**:
  - 🟩 Green = Start position
  - 🟥 Red = Goal position
  - 🔵 Cyan = A\* path
  - 🟣 Magenta = Dynamic A\* path
  - 🟢 Green = LPA\* path
  - 🟦 Dark Blue = Chosen path
  - 🟡 Yellow = Agent
  - ⬛ Black = Obstacles
- **Right-Side Information Panel**: Controls, metrics, and legend
- **Real-Time HUD**: Algorithm timings with performance data
- **Visual Feedback**: Mode indicators, override status, algorithm selection

### Intelligent Systems

- **Position Tracking**: Monitors last 8 positions for loop detection
- **Revisit Counter**: Detects 3+ visits to same position
- **Algorithm Comparator**: Parallel execution with performance tracking
- **Smart Selection**: Cost-based with dynamic algorithm preference
- **Automatic Warnings**: Console feedback with detailed metrics
- **Reset Capability**: Clear loop detection when user takes control

## 🧮 Algorithms

### A\* (A*) Pathfinding

**Initial Path Calculation**
- Optimal pathfinding using heuristic search
- Manhattan distance heuristic
- Guaranteed shortest path in static environment

**Formula:** `f(n) = g(n) + h(n)`
- `g(n)` = actual cost from start to n
- `h(n)` = heuristic estimate from n to goal
- `f(n)` = estimated total cost through n

**Characteristics:**
- ✅ Complete and optimal
- ✅ Admissible heuristic guarantees shortest path
- ❌ Must fully replan when environment changes
- Time Complexity: O(E log V) where E = edges, V = vertices
- Space Complexity: O(V)

### Dynamic A\* (D\* Lite)

**Incremental Replanning**
- Efficiently updates paths when obstacles change
- Only recalculates affected portions
- Key-based priority system for efficient updates
- Searches backward from goal to start

**Advantages:**
- ✅ 3-5× faster replanning than full A\*
- ✅ Ideal for dynamic environments
- ✅ Maintains optimality while being efficient
- ✅ Only updates inconsistent nodes

**Implementation:**
- Plans from goal to start (backward search)
- Two-key priority system: `[min(g,rhs) + h; min(g,rhs)]`
- Local repair of paths
- Consistency maintenance

**Performance:**
- Initial: 1-60ms (similar to A\*)
- Repair: 1-10ms (much faster than A\*)

### LPA\* (Lifelong Planning A\*)

**Incremental Heuristic Search**
- Foundation algorithm for D\* Lite
- Forward search from start to goal
- Maintains g-values and rhs-values
- Updates only locally inconsistent nodes

**Key Concepts:**
- **g-value**: Current cost estimate from start
- **rhs-value**: One-step lookahead value
- **Consistency**: node is consistent when g = rhs
- **Priority**: `[min(g,rhs) + h(start,s); min(g,rhs)]`

**Formula:**
```
rhs(s) = min_{s'∈Succ(s)} (g(s') + cost(s,s'))
```

**Characteristics:**
- ✅ Complete and optimal
- ✅ Incremental search
- ✅ Educational value (simpler than D\* Lite)
- ⚠️ Slower in practice due to forward search
- Time Complexity: O(C log V) for repair (C = affected cells)

**Performance:**
- Initial: 50-800ms (more nodes explored)
- Repair: 5-100ms (incremental updates)

## 🔬 Real-Time Comparator

### Overview
The **Real-Time Algorithm Comparator** runs all three algorithms simultaneously and provides comprehensive performance analysis.

### Features

**Parallel Execution:**
- All algorithms run on same grid
- Identical start and goal positions
- Real-time performance measurement
- Independent path computation

**Performance Metrics:**
- ⏱️ **Planning Time**: Milliseconds to compute path
- 📊 **Nodes Expanded**: Search space explored
- 💰 **Path Cost**: Total movement cost
- ✅ **Success**: Path found indicator

**Visual Comparison:**
```
Algorithm timings:
A*: 2 ms  (n=46) c=23 OPT
Dynamic A* (D* Lite): 1 ms  (n=97) c=23 OPT CHOSEN
LPA*: 56 ms  (n=34784) c=14 OPT
```

**Labels:**
- **OPT**: Algorithm found optimal (lowest cost) path
- **CHOSEN**: Algorithm currently being used by agent

### Intelligent Auto-Selection

**Selection Criteria (Priority Order):**

1. **Path Cost** (Primary)
   - Selects algorithm with minimum path cost
   - Cost difference threshold: 0.001

2. **Dynamic Capability** (Tie-Breaker)
   - Prefers Dynamic A\* and LPA\* over A\*
   - Rationale: Better for replanning scenarios

3. **Planning Time** (Final Tie-Breaker)
   - Selects fastest when all else equal

**Example:**
```
Scenario: Equal costs (c=23)
A*:        2ms → Not chosen (not dynamic)
D* Lite:   1ms → CHOSEN (dynamic + fastest)
LPA*:      3ms → Not chosen (slower than D*)
```

### Path Visualization

**Color Coding:**
- **Cyan (Light Blue)**: A\* path
- **Magenta (Pink)**: Dynamic A\* path
- **Green (Lime)**: LPA\* path
- **Dark Blue**: Chosen/active path (agent follows this)
- **Yellow**: Agent position

**Updated Legend:**
```
Legend:
  Green = Start
  Red = Goal
  Cyan = A* Path
  Magenta = D* Path
  Green = LPA* Path
  Blue = Chosen Path
  Yellow = Agent
  Black = Obstacle
```

### Toggle Control

**Comparator Toggle (T key):**
- **Enabled**: Runs all 3 algorithms, shows metrics
- **Disabled**: Uses single algorithm (faster)

**Benefits:**
- Compare algorithm performance
- Educational demonstration
- See trade-offs in real-time
- Understand when each excels

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
| **ENTER** | Move agent one step forward (replans after each move) |
| **M** | Switch between Manual/Auto mode |
| **T** | Toggle algorithm comparator ON/OFF ⭐NEW |
| **SPACEBAR** | Toggle manual override (Auto mode only) |
| **MOUSE** | Click cells to toggle obstacles (Manual mode) |
| **N** | Next level (after completion) |
| **R** | Reset to main menu |
| **ESC / Q** | Quit to menu |

### Algorithm Control ⭐NEW
| Key | Action |
|-----|--------|
| **T** | Enable/Disable real-time comparator |
| **A** | Switch to A\* algorithm (when comparator off) |
| **D** | Switch to Dynamic A\* (when comparator off) |
| **L** | Switch to LPA\* (when comparator off) |

### Menu Navigation
| Action | Method |
|--------|--------|
| Select mode | Click Manual or Auto button |
| Choose level | Click level 1-5 button |
| Select start | Click any free cell |
| Select goal | Click any free cell (not start) |

### Comparator Controls ⭐NEW

**When Comparator is ON (T key):**
- All 3 algorithms run automatically
- System selects optimal algorithm
- HUD shows metrics for all algorithms
- Colored paths overlay on grid
- Slightly slower (runs 3× algorithms)

**When Comparator is OFF:**
- Single algorithm runs (user-selected)
- Faster performance
- Single path shown
- No comparison metrics

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
    src/DynamicAStarPathfinder.cpp src/LPAStarPathfinder.cpp ^
    src/Agent.cpp src/SimpleRenderer.cpp ^
    -lopengl32 -lgdi32 -luser32 -lkernel32 -lwinmm ^
    -o MazeSolver.exe
```

**Source Files:**
- `main.cpp` - Entry point, window creation, render loop
- `Config.cpp` - Configuration loading
- `Grid.cpp` - Grid structure and cost management
- `Pathfinder.cpp` - Base pathfinder interface
- `AStarPathfinder.cpp` - Standard A* algorithm
- `DynamicAStarPathfinder.cpp` - D* Lite incremental search
- `LPAStarPathfinder.cpp` - LPA* forward incremental search ⭐NEW
- `Agent.cpp` - Agent control, pathfinding, comparator integration
- `SimpleRenderer.cpp` - OpenGL rendering, HUD, multi-path visualization

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
│   ├── LPAStarPathfinder.cpp       # LPA* implementation ⭐NEW
│   ├── Agent.cpp                   # Agent movement logic
│   └── SimpleRenderer.cpp          # OpenGL rendering
├── include/
│   ├── Config.h
│   ├── Grid.h
│   ├── Point.h
│   ├── Pathfinder.h
│   ├── AStarPathfinder.h
│   ├── DynamicAStarPathfinder.h
│   ├── LPAStarPathfinder.h         # LPA* header ⭐NEW
│   ├── Agent.h
│   └── SimpleRenderer.h
├── build_simple.bat                # Build script
├── config.txt                      # Configuration file
├── README.md
└── TECHNICAL_REPORT.md             # Comprehensive technical documentation
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
- Baseline algorithm for comparison

**DynamicAStarPathfinder** (`DynamicAStarPathfinder.cpp/.h`)
- Implements D* Lite algorithm
- Handles dynamic replanning
- Updates affected portions only
- More efficient for changing environments
- Backward incremental search

**LPAStarPathfinder** (`LPAStarPathfinder.cpp/.h`) ⭐NEW
- Implements Lifelong Planning A* (LPA*) algorithm
- Forward incremental search algorithm
- Maintains g-values and rhs-values
- Priority queue with two-key system (k1, k2)
- Efficient for repeated queries with minor changes
- Updates only affected nodes during replanning

**Agent** (`Agent.cpp/.h`)
- Controls agent movement
- Manages current position
- Stores and updates path
- Handles replanning triggers
- Integrates real-time comparator ⭐NEW
- Selects optimal algorithm dynamically ⭐NEW

**SimpleRenderer** (`SimpleRenderer.cpp/.h`)
- OpenGL-based rendering
- Draws grid, path, and agent
- Manages window and viewport
- Renders UI elements
- Multi-path visualization (cyan A*, magenta D*, green LPA*) ⭐NEW
- Real-time metrics HUD ⭐NEW

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
- See incremental replanning in action (D* Lite, LPA*) ⭐NEW
- Compare static vs. dynamic algorithms in real-time ⭐NEW
- Observe performance differences between forward and backward search ⭐NEW

**Problem Solving:**
- Path optimization strategies
- Handling dynamic environments
- Loop detection and resolution
- Trade-offs between algorithms
- Understanding incremental search principles ⭐NEW

**Programming Concepts:**
- Event-driven architecture
- State machine patterns
- OpenGL rendering with multi-path visualization ⭐NEW
- User input handling
- Priority queue-based algorithms ⭐NEW
- Real-time performance metrics tracking ⭐NEW

### Use Cases

- **University Projects**: AI/Algorithm courses
- **Research**: Pathfinding algorithm comparison
- **Teaching Tool**: Interactive demonstrations
- **Game Development**: Pathfinding prototyping
- **Algorithm Benchmarking**: Real-time performance analysis ⭐NEW

## 📊 Statistics

- **Total Lines of Code**: ~3,500+ ⭐UPDATED
- **Algorithms Implemented**: 3 (A*, D* Lite, LPA*) ⭐UPDATED
- **Source Files**: 9 ⭐UPDATED
- **Header Files**: 9 ⭐UPDATED
- **Build Time**: <5 seconds
- **Supported Grid Sizes**: 10×10 to 100×100
- **Heuristic Functions**: 3 (Manhattan, Euclidean, Octile)

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
- Disable comparator (T key) for single-algorithm mode ⭐NEW
- Disable VSync if needed
- Close background applications

**Loop detection too sensitive:**
- Increase position history size (line 68 in main.cpp)
- Adjust revisit threshold (line 83 in main.cpp)

**Comparator showing unexpected results:** ⭐NEW
- Press T to toggle comparator on/off
- Check algorithm metrics in HUD panel
- Verify obstacle placement isn't blocking all paths

## 🤝 Contributing

Contributions welcome! Areas for improvement:

- Additional pathfinding algorithms (Dijkstra, JPS, Theta*)
- More heuristic options
- Save/load maze layouts
- Replay functionality
- Performance metrics export ⭐UPDATED
- Additional themes/colors
- Path smoothing algorithms ⭐NEW

## 📄 License

This project is created for educational purposes. Feel free to use and modify for learning and teaching.

## 👥 Authors

- **AI Lab Project** - Dynamic Maze Solver with Real-Time Comparator ⭐UPDATED
- Course: 5th Semester AI Lab

## 🙏 Acknowledgments

- A* algorithm by Peter Hart, Nils Nilsson, and Bertram Raphael (1968)
- D* Lite by Sven Koenig and Maxim Likhachev (2002) ⭐NEW
- LPA* (Lifelong Planning A*) by Sven Koenig and Maxim Likhachev (2001) ⭐NEW
- D* Lite algorithm by Sven Koenig and Maxim Likhachev (2002)
- OpenGL for graphics rendering
- MinGW for Windows compilation

---

**⭐ Star this repository if you find it helpful!**

**📧 Report issues or suggest features in the Issues tab**

---

**⭐ Star this repository if you find it helpful!**

**📧 Report issues or suggest features in the Issues tab**

