#include <deque>
#include <iostream>

#include "AStar.h"
#include "Grid.h"
#include "NavProfile.h"
#include "Visualizer.h"

using Pathfinding::AStar;
using Pathfinding::Grid;
using Pathfinding::NavProfile;
using Pathfinding::TerrainFlags;

int main()
{
    constexpr int32_t gridWidth = 100;
    constexpr int32_t gridHeight = 10;

    size_t startIdx = 5;
    size_t endIdx = 399;

    // Build the level
    Grid grid(gridWidth, gridHeight);
    for (int32_t y = 0; y < gridHeight - 1; ++y)   // a wall with a gap at the bottom
        grid.SetTerrain(50, y, TerrainFlags::Wall);
    for (int32_t y = 5; y < 10; ++y)               // a pond the walker cannot cross
        grid.SetTerrain(25, y, TerrainFlags::Water);
    for (int32_t y = 0; y < gridHeight; ++y)       // a mud band that cannot be walked around
        for (int32_t x = 75; x <= 80; ++x)
            grid.SetTerrain(x, y, TerrainFlags::Mud);

    AStar aStar(grid);

    NavProfile walker;
    walker.blockedMask = TerrainFlags::Wall | TerrainFlags::Water;
    walker.SetTerrainCost(TerrainFlags::Mud, 300);  // Mud costs three times as much to walk through

    std::deque<size_t> path;
    if (!aStar.FindPath(startIdx, endIdx, path, walker))
    {
        std::cerr << "No path found from " << startIdx << " to " << endIdx << '\n';
        return 1;
    }

    // Visualise the final grid state
    Pathfinding::Debug::Visualizer::PrintGrid(grid, startIdx, endIdx, aStar.GetOpen(), aStar.GetClosed(), path);

    // The last closed node is the end node, so its g cost is the cost of the whole path
    std::cout << "Path cost: " << aStar.GetClosed().back().gCost << '\n';

    return 0;
}