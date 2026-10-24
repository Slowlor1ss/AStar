#include <deque>
#include <iostream>

#include "AStar.h"
#include "Visualizer.h"

int main()
{
    constexpr int32_t gridWidth = 100;
    constexpr int32_t gridHeight = 10;

    size_t startIdx = 5;
    size_t endIdx = 399;

    AStar aStar(gridWidth, gridHeight);

    std::deque<size_t> path;
    if (!aStar.FindPath(startIdx, endIdx, path))
    {
        std::cerr << "No path found from " << startIdx << " to " << endIdx << '\n';
        return 1;
    }

    // Visualise the final grid state
    Visualizer::PrintGrid(gridWidth, gridHeight, startIdx, endIdx, aStar.GetOpen(), aStar.GetClosed(), path);

    return 0;
}