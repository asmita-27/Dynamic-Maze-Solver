#include "RealtimeComparator.h"
#include "AStarPathfinder.h"
#include "DynamicAStarPathfinder.h"
#include "LPAStarPathfinder.h"
#include <chrono>

RealtimeComparator::RealtimeComparator() {}

std::vector<ComparatorEntry> RealtimeComparator::runAll(const Grid& grid, const Point& start, const Point& goal, bool eightDir) {
    std::vector<ComparatorEntry> results;

    // A*: baseline
    {
        AStarPathfinder pf;
        pf.setHeuristic("manhattan");
        pf.setMovementType(eightDir);

        auto t0 = std::chrono::high_resolution_clock::now();
        PathfindingResult r = pf.findPath(grid, start, goal);
        auto t1 = std::chrono::high_resolution_clock::now();
        r.planningTime = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0);

        ComparatorEntry e;
        e.name = pf.getName();
        e.result = r;
        results.push_back(e);
    }

    // Dynamic A* (D* Lite) - if available
    {
        DynamicAStarPathfinder pf;
        pf.setHeuristic("manhattan");
        pf.setMovementType(eightDir);

        auto t0 = std::chrono::high_resolution_clock::now();
        PathfindingResult r = pf.findPath(grid, start, goal);
        auto t1 = std::chrono::high_resolution_clock::now();
        r.planningTime = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0);

        ComparatorEntry e;
        e.name = pf.getName();
        e.result = r;
        results.push_back(e);
    }

    // LPA* - Lifelong Planning A*
    {
        LPAStarPathfinder pf;
        pf.setHeuristic("manhattan");
        pf.setMovementType(eightDir);

        auto t0 = std::chrono::high_resolution_clock::now();
        PathfindingResult r = pf.findPath(grid, start, goal);
        auto t1 = std::chrono::high_resolution_clock::now();
        r.planningTime = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0);

        ComparatorEntry e;
        e.name = pf.getName();
        e.result = r;
        results.push_back(e);
    }

    return results;
}
