#include <iostream>
#include <vector>
#include <string>
#include <deque>
#include <ranges>
#include <algorithm>

#include "Visualizer.h"

// This was a help source I used for ideas
// https://www.redblobgames.com/pathfinding/a-star/implementation.html
struct Node
{
    u_int32_t gCost = -1;
    u_int32_t hCost = -1;
    size_t idx = -1;

    size_t parentIdx = -1;

    u_int32_t Cost() const {return gCost + hCost;}
};

int32_t gridWidth = 50;
int32_t gridHeight = 50;

std::vector<Node> Open;
std::vector<Node> Closed;
std::vector<bool> IsClosed(gridWidth * gridHeight, false); // Prevents havng to check if neighbor is already in closed list

size_t startIdx = 5;
size_t endIdx = 199;

// Assuming y is vertical and x horizontal, and (0,0) is topLeft
size_t Get1DIdx(int32_t x, int32_t y)
{
    return gridWidth * y + x;
}

std::pair<int32_t, int32_t> Get2DIdx(size_t idx)
{
    int32_t x = idx % gridWidth;
    int32_t y = idx / gridWidth;
    return {x, y};
}

// Finds all neighbors for a idx,
// does not clear list, and returns the last valid idx
size_t fillNeighborList(size_t index, std::vector<size_t>& neighborList)
{
    __glibcxx_assert(!neighborList.size() < 8 || "NeighborList should at least have a size of 8 in order to be filled");

    auto[x, y] = Get2DIdx(index);

    size_t idx = 0;
    for (int32_t dy = -1; dy <= 1; ++dy)
    {
        for (int32_t dx = -1; dx <= 1; ++dx)
        {
            // Skip center aka ourselves
            if (dx == 0 && dy == 0)
                continue;

            // Get neighbor cords
            int32_t nx = x + dx;
            int32_t ny = y + dy;

            // Make sure its within our grid
            if (nx < 0 || nx >= gridWidth ||
                ny < 0 || ny >= gridHeight)
                continue;

            neighborList[idx] = (
                static_cast<size_t>(ny) * gridWidth +
                static_cast<size_t>(nx)
            );
            ++idx;
        }
    }
    return idx;
}

u_int32_t GetOctileDistance(size_t a, size_t b)
{
    auto [ax, ay] = Get2DIdx(a);
    auto [bx, by] = Get2DIdx(b);

    // Calculate absolute structural differences
    int32_t dx = std::abs(ax - bx);
    int32_t dy = std::abs(ay - by);

    // Branchless math https://www.geeksforgeeks.org/dsa/a-search-algorithm/ ( 1) Manhattan Distance )
    // Take the minimum of dx and dy as diagonal steps (cost 14)
    // and the remaining steps as straight lines (cost 10)
    return 10 * (dx + dy) + (14 - 2 * 10) * std::min(dx, dy); // (std::min does not suffer from branch prediction :D) 
}


u_int32_t GetDistanceBetweenNodes(size_t a, size_t b)
{
    return GetOctileDistance(a, b);
}

void ExploreNeghbors(size_t currentIdx, std::vector<size_t>& neighborList)
{
    size_t end = fillNeighborList(currentIdx, neighborList);
    u_int32_t lowestCost = -1u;
    Node lowestCostNode;
    for(int i = 0; i < end; i++)
    {
        size_t neighborIdx = neighborList[i];
        if (IsClosed[neighborIdx])
            continue;

        Node nNode{-1u, -1u, neighborIdx, currentIdx};
        nNode.gCost = GetDistanceBetweenNodes(nNode.idx, startIdx);
        nNode.hCost = GetDistanceBetweenNodes(nNode.idx, endIdx);
        
        // This might add nodes we already found in a previous iteration but thats fine we can deal with duplicates
        Open.emplace_back(nNode);
        
        // u_int32_t cost = nNode.gCost + nNode.hCost;
        //
        // if (cost < lowestCost)
        // {
        //     if (lowestCost != -1)
        //     {
        //         Open.push_back(lowestCostNode);
        //     }

        //     lowestCost = cost;
        //     lowestCostNode = nNode;
        // }
        // else
        //     Open.push_back(nNode);
    }

    //Closed.push_back(lowestCostNode);
}

std::deque<size_t> ReconstructPath()
{
    std::deque<size_t> path;

    const Node* next = &Closed.back();
    do
    {
        // We need to find the matchng idx with the owest cost as 
        // we simply re-add nodes when we find one with a lower cost whle looking for paths
        // so we could end up with multiple nodes of the same idx, 
        // and to ensure we take the correct parent we need the cheapest one
        auto matching_nodes = Closed | std::views::filter([next](const Node& n) { 
            return n.idx == next->parentIdx; 
        });

        auto it = std::ranges::min_element(matching_nodes, [](const Node& a, const Node& b) {
            return a.Cost() < b.Cost();
        });

        path.emplace_front(next->idx);
        next = &(*it);
    } while (next->parentIdx != -1);
    
    return std::move(path);
}

int main()
{
    std::vector<size_t> neighborList(8);
    Node startNode = {0, 0, startIdx};
    Closed.emplace_back(startNode);
    IsClosed[startIdx] = true;

    uint32_t nextCheapestIdx = startIdx;
    do
    {
        ExploreNeghbors(nextCheapestIdx, neighborList);
        
        auto cheapestOpenit = std::min_element(Open.begin(), Open.end(), 
            [](const Node& a, const Node& b) {
                return a.Cost() < b.Cost();
            });

        nextCheapestIdx = (*cheapestOpenit).idx;
        Closed.emplace_back(std::move(*cheapestOpenit));
        IsClosed[nextCheapestIdx] = true;
        
        //Open.erase(cheapestOpenit);
        // Swap and pop!
        *cheapestOpenit = std::move(Open.back());
        Open.pop_back(); 
    
    } while (Closed.back().idx != endIdx);
    
    auto path = ReconstructPath();

    // Visualise the final grid state
    Visualizer::PrintGrid(gridWidth, gridHeight, startIdx, endIdx, Open, Closed, path);

    return 0;
}
