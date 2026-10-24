#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <vector>
#include <deque>
#include <iostream>
#include <unordered_set>

// Forward declaration of Node structure from main file to avoid cyclic dependencies
struct Node;

class Visualizer {
public:
    template <typename TNodeContainer>
    static void PrintGrid(int32_t width, int32_t height, 
                          size_t startIdx, size_t endIdx,
                          const TNodeContainer& openList, 
                          const TNodeContainer& closedList,
                          const std::deque<size_t>& path) 
    {
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
                size_t currentIdx = static_cast<size_t>(width * y + x);

                if (currentIdx == startIdx) {
                    std::cout << "S"; // Start
                } else if (currentIdx == endIdx) {
                    std::cout << "E"; // End
                } else if (pathIndices.contains(currentIdx)) {
                    std::cout << "x"; // Actual path taken
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
        
        std::cout << "Legend: S=Start | E=End | x=Path | .=Closed | ?=Open\n\n";
    }
};

#endif // VISUALIZER_H
