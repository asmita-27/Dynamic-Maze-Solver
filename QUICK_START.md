# Quick Start Guide

## Running the Application

1. **Build the project:**
   ```
   build_simple.bat
   ```
   Or run: `MazeSolver.exe` if already built

2. **Initial Setup (Console):**
   - Choose obstacle mode:
     - `1` - Manual (you place obstacles by clicking)
     - `2` - Automatic (random obstacles generated)

3. **Set Start Position:**
   - Enter coordinates, for example: `0 0`
   - For a 10x10 grid (Level 1), valid range is 0-9 for both x and y

4. **Set Goal Position:**
   - Enter coordinates, for example: `9 9`
   - Must be different from start position

## Gameplay

### Step-by-Step Movement
- **Press Enter or Right Arrow** to move ONE step
- After each step, the path is **automatically recalculated** from your new position
- This shows you the optimal path from wherever you are

### Adding Obstacles (Manual Mode Only)
- **Left Click** on any grid cell to toggle obstacles
- Cannot place obstacles on start/goal positions
- Path recalculates automatically when obstacles change

### Level Progression
- Reach the goal to complete the level
- **Press N** to advance to next level
- Choose obstacle mode again for the new level
- 5 levels total (EASY → MEDIUM → HARD → VERY HARD → EXPERT)

## Example Play Session

```
========== LEVEL 1 ==========
Difficulty: EASY
Grid Size: 10x10
Obstacles: 10%
=================================

Enter START position (x y): 0 0
Enter GOAL position (x y): 9 9

[Press Enter to take first step]

[→] Taking one step... moved to (1,0)
[✓] Recalculating optimal path from new position...

[Press Enter again]

[→] Taking one step... moved to (2,0)
[✓] Recalculating optimal path from new position...

[Keep pressing Enter until you reach the goal]

🎉🎉🎉 CONGRATULATIONS! 🎉🎉🎉
You reached the goal!
Press N to advance to Level 2

[Press N for next level]
```

## All Controls

| Key | Action |
|-----|--------|
| **Enter / Right Arrow** | Move one step + recalculate path |
| **Space** | Force path recalculation |
| **N** | Next level (after reaching goal) |
| **R** | Reset current level |
| **A** | Switch to A* algorithm |
| **D** | Switch to Dynamic A* algorithm |
| **Left Click** | Toggle obstacle (manual mode) |
| **ESC** | Exit application |

## Tips

1. **Start Simple**: Begin with Level 1 (10x10) to get familiar
2. **Watch the Console**: Shows detailed path recalculation info
3. **Experiment**: Try both manual and automatic obstacle modes
4. **Compare Algorithms**: Switch between A* (press A) and Dynamic A* (press D)
5. **Strategic Planning**: In manual mode, add obstacles to challenge yourself

## Level Details

| Level | Grid | Obstacles |
|-------|------|-----------|
| 1 | 10x10 | 10% |
| 2 | 15x15 | 15% |
| 3 | 20x20 | 20% |
| 4 | 25x25 | 25% |
| 5 | 30x30 | 30% |

Good luck solving the mazes! 🎮
