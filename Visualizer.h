#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <iostream>
#include <unordered_set>

#include "Grid.h"

namespace Pathfinding::Debug
{

class Visualizer 
{
public:
    // The node containers are templated so the visualizer only needs nodes to have an `idx`,
    // it does not have to know the Node type
    template <typename TNodeContainer>
    static void PrintGrid(const Grid& grid,
                          size_t startIdx, size_t endIdx,
                          const TNodeContainer& openList, 
                          const TNodeContainer& closedList,
                          const std::deque<size_t>& path) 
    {
        const int32_t width = grid.GetWidth();
        const int32_t height = grid.GetHeight();

        // Collect idx's for lookups while drawing
        std::unordered_set<size_t> openIndices;
        for (const auto& n : openList) {
            openIndices.insert(n.idx);
        }

        std::unordered_set<size_t> closedIndices;
        for (const auto& n : closedList) {
            closedIndices.insert(n.idx);
        }

        std::unordered_set<size_t> pathIndices(path.begin(), path.end());

        // Draw the layout to terminal console
        std::cout << "\n--- A* Viz (" << width << "x" << height << ") ---\n";
        
        // Some lil pluses for the corners (fancy)
        std::cout << "+";
        for (int32_t x = 0; x < width; ++x) std::cout << "-";
        std::cout << "+\n";

        for (int32_t y = 0; y < height; ++y) 
        {
            std::cout << "|"; // Vertical boundaries left
            
            for (int32_t x = 0; x < width; ++x) 
            {
                const size_t currentIdx = grid.Get1DIdx(x, y);
                const TerrainFlags terrain = grid.GetTerrain(currentIdx);

                if (currentIdx == startIdx) {
                    std::cout << "S"; // Start
                } else if (currentIdx == endIdx) {
                    std::cout << "E"; // End
                } else if (pathIndices.contains(currentIdx)) {
                    std::cout << "x"; // Actual path taken
                } else if (terrain != TerrainFlags::None) {
                    // Terrain wins over the search markers so passable terrain
                    // (mud, water, ...) stays visible, blocked cells are never searched anyway
                    std::cout << TerrainGlyph(terrain);
                } else if (closedIndices.contains(currentIdx)) {
                    std::cout << "."; // Processed Graveyard
                } else if (openIndices.contains(currentIdx)) {
                    std::cout << "?"; // Neigbors we ignored
                } else {
                    std::cout << " "; // Unexplored
                }
            }
            std::cout << "|\n"; // Vertical boundaries right
        }

        // Bottom border
        std::cout << "+";
        for (int32_t x = 0; x < width; ++x) std::cout << "-";
        std::cout << "+\n";
        
        std::cout << "Legend: S=Start | E=End | x=Path | .=Closed | ?=Open\n"
                  << "        #=Wall | ~=Water | %=Mud | ^=Lava\n\n";
    }

private:
    // Keep in sync with TerrainFlags (and the legend above) when you add terrain types
    static char TerrainGlyph(TerrainFlags terrain)
    {
        // A cell can carry several flags, draw the most important one
        if (HasAnyFlag(terrain, TerrainFlags::Wall))  return '#';
        if (HasAnyFlag(terrain, TerrainFlags::Lava))  return '^';
        if (HasAnyFlag(terrain, TerrainFlags::Water)) return '~';
        if (HasAnyFlag(terrain, TerrainFlags::Mud))   return '%';
        return '!'; // Flags we have no glyph for yet
    }
};

}