# Dynamic Maze Solver - New Features

## Overview
The maze solver now includes an enhanced interactive experience with progressive difficulty levels, user choices for obstacle generation, and automatic path recalculation after each step.

## New Features Implemented

### 1. **Initial Setup Menu**
When the application starts, you'll be prompted to choose:
- **Manual Obstacle Mode**: You click on the grid to place obstacles yourself
- **Automatic Obstacle Mode**: Obstacles are randomly generated based on the level difficulty

### 2. **Custom Start and Goal Positions**
For each level, you can:
- Enter custom X and Y coordinates for the START position
- Enter custom X and Y coordinates for the GOAL position
- The system validates your inputs to ensure they're within grid bounds
- Start and goal positions cannot be the same

### 3. **Progressive Difficulty Levels (1-5)**
The game now features 5 progressive levels with increasing difficulty:

| Level | Difficulty | Grid Size | Obstacle Density |
|-------|-----------|-----------|------------------|
| 1     | EASY      | 10x10     | 10%             |
| 2     | MEDIUM    | 15x15     | 15%             |
| 3     | HARD      | 20x20     | 20%             |
| 4     | VERY HARD | 25x25     | 25%             |
| 5     | EXPERT    | 30x30     | 30%             |

### 4. **Step-by-Step Movement with Automatic Replanning**
- Press **Enter** or **Right Arrow** to move ONE step at a time
- After EACH step, the optimal path is automatically recalculated from your new position
- This ensures you always see the best path forward from your current location
- Watch the console to see the path recalculation messages

### 5. **Level Progression System**
- Complete a level by reaching the goal
- Press **N** to advance to the next level
- Each time you start or reset a level, you can choose obstacle mode again
- Track your progress through all 5 levels

## Controls

### Movement
- **Enter / Right Arrow**: Take one step forward (with automatic path recalculation)
- **Space**: Force manual path recalculation

### Level Management
- **N**: Next level (available after reaching goal)
- **R**: Reset current level

### Obstacle Management
- **Left Click**: Toggle obstacles on the grid (manual mode only)
  - Cannot place obstacles on start or goal positions
  - Automatically triggers path recalculation

### Algorithm Selection
- **A**: Switch to A* algorithm
- **D**: Switch to Dynamic A* (D* Lite) algorithm

### Other
- **ESC**: Exit application

## How to Play

1. **Start the Application**
   - Choose obstacle mode (1 for Manual, 2 for Automatic)

2. **Set Start and Goal**
   - Enter coordinates for start position (e.g., `0 0`)
   - Enter coordinates for goal position (e.g., `9 9`)

3. **Navigate the Maze**
   - Press Enter/Right Arrow to move one step at a time
   - Watch the path recalculate after each step
   - If in manual mode, click to add obstacles and watch the path adapt

4. **Reach the Goal**
   - When you reach the goal, you'll see a congratulations message
   - Press N to advance to the next level

5. **Progress Through Levels**
   - Complete all 5 levels, each getting progressively harder
   - Larger grids and more obstacles in higher levels

## Technical Details

### Automatic Path Recalculation
- The agent automatically recalculates the optimal path after each move
- This is done by setting the `needsReplanning_` flag after each step
- The path is recalculated before the next move is allowed

### Smart Obstacle Placement
- In manual mode, you cannot place obstacles on start/goal positions
- Placing obstacles triggers immediate path recalculation
- The pathfinder will adapt the route in real-time

### Algorithm Support
- **A* Algorithm**: Complete pathfinding from scratch each time
- **Dynamic A* (D* Lite)**: Efficient path repair when obstacles change
- Both algorithms recalculate after each step to show optimal path

## Console Feedback

The console provides detailed feedback:
- `[→] Taking one step...` - When you press Enter
- `[✓] Recalculating optimal path from new position...` - After each move
- `🎉 CONGRATULATIONS!` - When you reach the goal
- Level information with grid size and difficulty
- Algorithm and movement statistics

## Example Gameplay

```
Level 1 (EASY - 10x10):
Enter START: 0 0
Enter GOAL: 9 9

[→] Taking one step... moved to (1,0)
[✓] Recalculating optimal path from new position...

[→] Taking one step... moved to (2,0)
[✓] Recalculating optimal path from new position...

... continue until reaching goal ...

🎉 CONGRATULATIONS! You reached the goal!
Press N to advance to Level 2
```

## Benefits of New Features

1. **Educational**: Learn how pathfinding algorithms adapt to changes in real-time
2. **Interactive**: Full control over maze creation and navigation
3. **Progressive**: Build skills from easy to expert levels
4. **Transparent**: See the path recalculation happen after every move
5. **Flexible**: Choose between manual and automatic obstacle generation

Enjoy exploring pathfinding with your new dynamic maze solver! 🎮🗺️
