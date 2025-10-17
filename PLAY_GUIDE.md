# 🎮 Quick Start - Interactive Maze Solver

## What You'll See

### Game Window (Right)
- **15x15 grid** displayed with some obstacles
- Light gray cells = walkable spaces
- Black cells = obstacles
- This is just a sample - it will change when you start

### Console Window (Left)
- Instructions and feedback
- Real-time game status
- Step-by-step guidance

## 🚀 How to Play (Step by Step)

### Step 1: Choose Your Mode
**Look at the GAME WINDOW** (not console) and press:
- **Press 1** = MANUAL mode (you click to toggle obstacles)
- **Press 2** = AUTO mode (obstacles change automatically)

💡 **Make sure the game window has focus (click on it if needed)**

### Step 2: Select START
- **CLICK on any cell** in the grid
- The cell will turn **GREEN**
- Console will confirm your selection

### Step 3: Select GOAL
- **CLICK on a different cell** in the grid
- The cell will turn **RED**
- Cannot be the same as START

### Step 4: Navigate!
- **Blue path** appears showing optimal route
- **Yellow circle** appears at START (that's you!)
- **Press ENTER or SPACE** to move one step
- Path recalculates after each move

### Step 5: Reach the Goal!
- Navigate to the RED cell
- **Press N** for next level
- Try all 5 levels!

## 🎨 Visual Guide

```
Game Window Layout:
┌─────────────────────────────────┐
│  [Grid with obstacles]          │
│                                  │
│  ⬜ Light Gray = Walkable        │
│  ⬛ Black = Obstacle             │
│  🟩 Green = START (your click)  │
│  🟥 Red = GOAL (your click)     │
│  🔵 Blue line = Optimal path    │
│  🟡 Yellow circle = You!        │
│                                  │
└─────────────────────────────────┘
```

## 🎯 Color Legend

| Color | Meaning |
|-------|---------|
| 🟩 **Bright Green** | START position |
| 🟥 **Bright Red** | GOAL position |
| ⬛ **Black** | Obstacles (blocked) |
| ⬜ **Light Gray** | Free space (walkable) |
| 🔵 **Blue Line** | Optimal path to goal |
| 🟡 **Yellow Circle** | Your current position |

## 🕹️ Controls Summary

### Mode Selection (First Step)
- **1** - Manual mode
- **2** - Auto mode

### Mouse
- **Left Click** - Select START → GOAL → Toggle obstacles (manual mode)

### Keyboard
- **Enter** or **Space** - Move one step
- **N** - Next level (after winning)
- **R** - Reset level
- **A** - Switch to A* algorithm
- **D** - Switch to Dynamic A*
- **ESC** - Exit

## 🎮 Gameplay Modes

### MANUAL Mode (Press 1)
- ✅ Click cells during play to toggle obstacles
- ✅ Path recalculates when you change obstacles
- ✅ Full control over maze layout
- ✅ Strategic planning possible

### AUTO Mode (Press 2)
- ✅ After each move, 1-3 obstacles change randomly
- ✅ Path adapts automatically
- ✅ More challenging and unpredictable
- ✅ Tests real-time adaptation

## 📊 Progressive Levels

| Level | Grid Size | Obstacles | Difficulty |
|-------|-----------|-----------|------------|
| 1 | 10×10 | 10% | 😊 EASY |
| 2 | 15×15 | 15% | 🙂 MEDIUM |
| 3 | 20×20 | 20% | 😐 HARD |
| 4 | 25×25 | 25% | 😰 VERY HARD |
| 5 | 30×30 | 30% | 😱 EXPERT |

## 💡 Pro Tips

1. **Window Focus**: Make sure game window is active (not console)
2. **Click Inside Grid**: Click on the actual grid cells, not outside
3. **Watch Console**: It gives helpful feedback about your clicks
4. **Path Updates**: Blue path shows best route from current position
5. **Start Simple**: Try level 1 in manual mode first
6. **Experiment**: Try both A* and Dynamic A* algorithms (press A or D)

## 🐛 Troubleshooting

**Q: I don't see the grid**
- A: A sample grid should appear immediately. Try resizing the window.

**Q: My clicks aren't working**
- A: Make sure you clicked on the game window first (to give it focus)
- Check console - it shows if clicks are detected and where

**Q: Colors don't match description**
- A: Grid should show:
  - Light gray = free space
  - Black = obstacles
  - Green = START (after you click)
  - Red = GOAL (after you click)

**Q: Can't select start/goal**
- A: First press 1 or 2 to select mode
- Then click on grid cells
- Console shows exact position you clicked

**Q: Path doesn't show**
- A: Path only appears after you select both START and GOAL
- It's a blue line connecting cells

## 📝 Example Session

```
1. Game starts → See 15×15 grid with obstacles
2. Console says "Press [1] or [2]"
3. Press 2 (for AUTO mode)
4. Grid changes to match selected level
5. Console says "CLICK on cell for START"
6. Click top-left corner → Turns GREEN
7. Console says "CLICK on cell for GOAL"
8. Click bottom-right corner → Turns RED
9. Blue path appears + Yellow circle at START
10. Press Enter → Yellow circle moves one step
11. Path recalculates + some obstacles change (auto mode)
12. Keep pressing Enter until you reach RED cell
13. "CONGRATULATIONS!" appears
14. Press N for next level!
```

## 🎯 Learning Features

- **Pathfinding Visualization**: See A* algorithm in action
- **Dynamic Replanning**: Watch path adapt to changes
- **Algorithm Comparison**: Compare A* vs Dynamic A*
- **Progressive Difficulty**: Build skills gradually
- **Interactive Learning**: Hands-on maze modification

---

**Ready to play?** 🎮

1. Make sure game window is visible
2. Press **1** or **2** in the game window
3. Click to select START and GOAL
4. Press Enter to navigate!

Have fun! 🎉
