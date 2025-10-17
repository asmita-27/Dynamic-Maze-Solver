# Dynamic Maze Solver - Usage Guide

## 🎮 Complete User Guide

### Main Features
- **Professional UI with clickable buttons**
- **Two modes**: Manual obstacle mode and Auto obstacle mode
- **5 progressive difficulty levels** (10x10 to 30x30 grids)
- **Path recalculation** after every step
- **A* and Dynamic A*** pathfinding algorithms

---

## 🚀 How to Use

### 1. **Main Menu** (Front Page)
When you start the application, you'll see a beautiful front page with:

**Left Panel - CONTROLS:**
- Mouse clicks for all interactions
- Click buttons to select options
- Click grid cells to select start/goal
- Click obstacles to toggle them
- ENTER key to move agent
- R key to reset

**Right Panel - GAME INFO:**
- 5 Difficulty Levels
- Grids from 10x10 to 30x30
- A* & Dynamic A* algorithms
- Real-time path recalculation
- Progressive challenge system

**Action:** Click the green **"START"** button at the bottom to begin!

---

### 2. **Mode Selection**
After clicking START, you'll see two large buttons:

#### 🟢 **MANUAL MODE** (Green button - Left)
- You manually place obstacles by clicking
- Full control over maze environment
- Great for testing specific scenarios
- Click obstacles on/off during gameplay

#### 🔴 **AUTO MODE** (Red button - Right)
- Obstacles change randomly after each move
- Dynamic environment simulation
- Tests adaptive pathfinding
- More challenging gameplay

**Action:** Click your preferred mode button!

---

### 3. **Level Selection**
Choose your difficulty level by clicking one of 5 buttons:

1. **🟢 EASY** - 10x10 Grid (10% obstacles)
2. **🟡 MEDIUM** - 15x15 Grid (15% obstacles)
3. **🟠 HARD** - 20x20 Grid (20% obstacles)
4. **🔴 VERY HARD** - 25x25 Grid (25% obstacles)
5. **⚫ EXPERT** - 30x30 Grid (30% obstacles)

**Action:** Click the level button you want to play!

---

### 4. **Select Start Position**
The grid will appear. Now:
- **Click any white cell** on the grid to set the START position
- The cell will turn **GREEN** 
- Instructions appear on the right side

---

### 5. **Select Goal Position**
After selecting start:
- **Click another white cell** on the grid to set the GOAL position
- The cell will turn **RED**
- Cannot be the same as start position
- The blue path will automatically appear!

---

### 6. **Gameplay**

#### Controls During Play:
- **ENTER** - Move agent one step forward along the path
- **MOUSE CLICK** (Manual mode only) - Toggle obstacles on/off
- **R** - Reset to main menu
- **ESC** - Quit application

#### What Happens:
- **After each move**, the agent recalculates the optimal path
- **In MANUAL mode**: Click cells to add/remove obstacles
- **In AUTO mode**: Random obstacles appear/disappear after each move
- The blue line shows the current optimal path
- Yellow circle is your agent moving toward the red goal

#### Visual Legend:
- 🟢 **Green** = Start position
- 🔴 **Red** = Goal position  
- 🔵 **Blue line** = Optimal path
- 🟡 **Yellow circle** = Agent (you)
- ⚫ **Black** = Obstacles
- ⚪ **White** = Walkable cells

---

### 7. **Level Complete**
When you reach the goal:
- **"GOAL REACHED!"** message appears
- **Press N** - Advance to next level
- **Press R** - Return to main menu

---

## 🎯 Tips for Success

1. **Start with EASY level** to understand mechanics
2. **Manual mode** is easier for beginners
3. **Auto mode** is more challenging and realistic
4. Watch how the **path recalculates** after obstacles change
5. The pathfinding is **optimal** - it always finds the shortest path!

---

## 🐛 Auto Mode Fix
**IMPORTANT:** The auto mode now works correctly!
- The agent's movement mode is properly set when you select AUTO
- Random obstacles will appear/disappear after each ENTER press
- The path automatically recalculates to avoid new obstacles

---

## 🔧 Technical Details

### Build Command:
```bash
.\build_simple.bat
```

### Project Structure:
- **Main Menu** → **Mode Selection** → **Level Selection** → **Gameplay**
- All states properly separated
- Button-based navigation
- Visual feedback for all actions

### Algorithms:
- **A* Algorithm**: Classic optimal pathfinding
- **Dynamic A* (D* Lite)**: Efficient replanning when environment changes
- Both ensure shortest path is always shown

---

## 💡 Why This Project is Awesome

✅ **Click-based UI** - No typing, just clicks!  
✅ **Progressive difficulty** - 5 levels from easy to expert  
✅ **Visual feedback** - Everything is color-coded and clear  
✅ **Two game modes** - Manual control or random challenges  
✅ **Real AI algorithms** - A* and Dynamic A* in action  
✅ **Path recalculation** - See adaptive pathfinding in real-time  
✅ **Professional design** - Clean, modern interface  

---

## 🎓 Educational Value

This project demonstrates:
- **Graph algorithms** (A*, D* Lite)
- **Heuristic search** (Manhattan distance)
- **Dynamic replanning** (environment changes)
- **OpenGL graphics** (2D rendering)
- **Game state management** (FSM pattern)
- **Event-driven programming** (mouse/keyboard input)
- **C++ best practices** (smart pointers, RAII)

---

## 📝 Notes

- Text rendering uses Windows GDI (may vary by system)
- All core functionality works via visual indicators
- VSync enabled for smooth rendering
- 30 FPS cap for stability
- Render-on-demand for efficiency

Enjoy playing and learning! 🚀
