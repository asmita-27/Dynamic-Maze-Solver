@echo off
echo ==========================================
echo  DYNAMIC MAZE SOLVER
echo  Simple Build - Direct G++ Compilation
echo ==========================================

rem Clean previous build
if exist "MazeSolver.exe" del "MazeSolver.exe"

echo Building with g++ for MinGW...

g++ -std=c++14 ^
    -Wall -Wextra ^
    -O2 ^
    -Iinclude ^
    -mwindows ^
    -static-libgcc -static-libstdc++ ^
    src/main.cpp ^
    src/Config.cpp ^
    src/Grid.cpp ^
    src/Pathfinder.cpp ^
    src/AStarPathfinder.cpp ^
    src/DynamicAStarPathfinder.cpp ^
    src/Agent.cpp ^
    src/SimpleRenderer.cpp ^
    -lopengl32 -lgdi32 -luser32 -lkernel32 -lwinmm ^
    -o MazeSolver.exe

if exist "MazeSolver.exe" (
    echo.
    echo ==========================================
    echo  BUILD SUCCESSFUL!
    echo ==========================================
    echo.
    echo ✅ A* and Dynamic A* algorithms
    echo ✅ Real-time pathfinding and replanning
    echo ✅ Interactive obstacle editing
    echo ✅ Scheduled and random events
    echo ✅ OpenGL visualization
    echo.
    echo Starting Dynamic Maze Solver...
    MazeSolver.exe
) else (
    echo ❌ Build failed!
    echo.
    echo Troubleshooting:
    echo 1. Ensure g++ is in PATH
    echo 2. Verify OpenGL libraries are available
    echo 3. Check MinGW installation
    pause
)
