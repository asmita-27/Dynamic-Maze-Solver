# Click-Based Interactive Maze Solver - User Guide

## 🎮 New Interactive Features!

Your maze solver now has a fully interactive, click-based interface! No more typing coordinates in the terminal - everything is done by clicking on the grid.

## 🚀 How to Play

### Step 1: Choose Your Mode
When you start the application, you'll see a menu in the console:
- **Press [1]** for **MANUAL mode** - You control obstacles by clicking
- **Press [2]** for **AUTO mode** - Obstacles change automatically after each move

### Step 2: Select START Position
- **CLICK on any grid cell** to mark it as your START position
- The cell will turn **GREEN**
- You'll see a confirmation in the console

### Step 3: Select GOAL Position
- **CLICK on a different grid cell** to mark it as your GOAL position
- The cell will turn **RED**
- Cannot be the same as START

### Step 4: Navigate the Maze
- **Press ENTER or SPACE** to move one step toward the goal
- After each step, the optimal path is automatically recalculated
- Watch the path update in real-time!

#### In MANUAL Mode:
- **CLICK on grid cells** to toggle obstacles ON/OFF
- Path recalculates immediately when you add/remove obstacles
- Full control over the maze layout

#### In AUTO Mode:
- After each move, 1-3 random obstacles toggle automatically
- Creates dynamic challenge as you navigate
- Path adapts to the changing environment

### Step 5: Complete the Level
- When you reach the goal: 🎉 **CONGRATULATIONS!**
- **Press N** to advance to the next level
- **Press R** to reset and try again

## 🎯 Progressive Difficulty

### Level 1: EASY
- Grid: 10x10
- Obstacles: 10%
- Perfect for learning!

### Level 2: MEDIUM
- Grid: 15x15
- Obstacles: 15%
- Getting trickier...

### Level 3: HARD
- Grid: 20x20
- Obstacles: 20%
- Serious challenge!

### Level 4: VERY HARD
- Grid: 25x25
- Obstacles: 25%
- Expert territory

### Level 5: EXPERT
- Grid: 30x30
- Obstacles: 30%
- Ultimate challenge! 🏆

## 🎮 Complete Controls

| Input | Action | When Available |
|-------|--------|----------------|
| **[1]** | Select MANUAL mode | Menu screen |
| **[2]** | Select AUTO mode | Menu screen |
| **Left Click** | Select START/GOAL or toggle obstacle | Start/Goal selection or Playing (manual mode) |
| **Enter/Space** | Move one step | During gameplay |
| **N** | Next level | After completing level |
| **R** | Reset level | Anytime |
| **A** | Switch to A* algorithm | Anytime |
| **D** | Switch to Dynamic A* | Anytime |
| **ESC** | Exit game | Anytime |

## 📋 Gameplay Flow

```
START
  ↓
[Console Menu Appears]
  ↓
Press 1 or 2
  ↓
[Grid Appears with Obstacles]
  ↓
CLICK to Select START (Green)
  ↓
CLICK to Select GOAL (Red)
  ↓
[Path Calculated & Shown]
  ↓
┌────────────────────────────┐
│   GAMEPLAY LOOP            │
│                            │
│   Press Enter              │
│   → Move 1 Step            │
│   → [AUTO: Toggle          │
│      obstacles randomly]   │
│   → Recalculate Path       │
│   → Show Updated Path      │
│                            │
│   [MANUAL: Click to        │
│    toggle obstacles        │
│    anytime]                │
│                            │
│   Reached Goal? ──No───┐   │
│        │ Yes            │   │
└────────┼────────────────┘   │
         ↓                    ↓
   🎉 Level Complete!      Continue...
         ↓
   Press N for Next Level
```

## 🎨 Visual Guide

### Grid Colors:
- **Gray**: Free space (walkable)
- **Black**: Obstacle (blocked)
- **Green**: START position
- **Red**: GOAL position
- **Blue Path**: Optimal route from current position to goal
- **Yellow Circle**: Your current position (agent)

## 💡 Pro Tips

1. **Plan Ahead**: Look at the initial obstacles before selecting start/goal
2. **Strategic Placement**: In manual mode, try placing obstacles to create interesting mazes
3. **Watch the Path**: The blue path shows your optimal route - it updates after every move!
4. **Auto Mode Challenge**: In auto mode, obstacles change unpredictably - can you adapt?
5. **Algorithm Comparison**: Try both A* (press A) and Dynamic A* (press D) to see the difference
6. **Start Simple**: Begin with Level 1 to understand the mechanics
7. **Take Your Time**: It's turn-based - think before each move!

## 🔍 Example Play Session

```
=== CONSOLE OUTPUT ===

=========================================
    DYNAMIC MAZE SOLVER
    A* and Dynamic A* (D* Lite)
=========================================

=== PATH CHANGE MODE ===
Press [1] for MANUAL mode - You toggle obstacles by clicking
Press [2] for AUTO mode - Obstacles change randomly after each move

[You press 2]

✓ AUTO mode selected
Obstacles will change randomly after each move

========== LEVEL 1 ==========
Difficulty: EASY
Grid Size: 10x10
Obstacles: 10%
=================================

🎯 CLICK on a grid cell to select START position

[You click on top-left corner]

✓ START selected at (0,0)

🎯 CLICK on a grid cell to select GOAL position

[You click on bottom-right corner]

✓ GOAL selected at (9,9)

▶ LEVEL 1 START!
Mode: AUTO - Obstacles change automatically
Press ENTER or SPACE to move one step at a time

[You press Enter]

[→] Taking one step...
Agent: Moved to (1,0) [17 steps remaining]
  🔄 Auto-toggled obstacle at (3,5)
  🔄 Auto-toggled obstacle at (7,2)
[✓] Recalculating optimal path from new position...
Agent: Path planned successfully - 18 steps

[Continue pressing Enter...]

🎉🎉🎉 CONGRATULATIONS! 🎉🎉🎉
You reached the goal!
Press N to advance to Level 2
```

## 🆘 Troubleshooting

**Q: I clicked but nothing happened**
- A: Make sure you're in the right state (selecting start/goal or playing)

**Q: Can't toggle obstacles**
- A: Only works in MANUAL mode during gameplay
- Can't toggle START or GOAL positions

**Q: Path seems wrong**
- A: Press Space to force recalculation
- Try switching algorithms with A or D

**Q: How do I change modes mid-game?**
- A: Press R to reset and choose a new mode

## 🎯 Learning Objectives

This interactive maze solver teaches:
- **Pathfinding Algorithms**: See A* and D* Lite in action
- **Dynamic Planning**: Watch how paths adapt to changes
- **Problem Solving**: Navigate increasingly complex mazes
- **Algorithm Comparison**: Understand different approaches
- **Real-time Adaptation**: Handle changing environments (auto mode)

## 🏆 Challenge Yourself

- ✅ Complete all 5 levels in MANUAL mode
- ✅ Complete all 5 levels in AUTO mode
- ✅ Try to use minimum steps (follow the path exactly)
- ✅ Create complex mazes in manual mode
- ✅ Compare A* vs Dynamic A* performance

## 📊 What You'll See

### Console Feedback:
- Mode selection confirmations
- Position selections
- Move-by-move progress
- Obstacle toggle notifications
- Path recalculation updates
- Level completion messages
- Algorithm switch confirmations

### Visual Feedback:
- Grid with all cells and obstacles
- Highlighted START (green) and GOAL (red)
- Real-time path visualization (blue)
- Agent position (yellow circle)
- Obstacle changes (instant)

---

**Have fun solving mazes!** 🎮🗺️

Press **1** or **2** to begin your adventure!
