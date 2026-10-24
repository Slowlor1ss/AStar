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

    AStar aStar(grid);

    NavProfile walker;
    walker.blockedMask = TerrainFlags::Wall | TerrainFlags::Water;

    std::deque<size_t> path;
    if (!aStar.FindPath(startIdx, endIdx, path, walker))
    {
        std::cerr << "No path found from " << startIdx << " to " << endIdx << '\n';
        return 1;
    }

    // Visualise the final grid state
    Pathfinding::Debug::Visualizer::PrintGrid(grid, startIdx, endIdx, aStar.GetOpen(), aStar.GetClosed(), path);

    return 0;
}