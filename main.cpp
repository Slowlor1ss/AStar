#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "Visualizer.h"

// This was a help source I used for ideas
// https://www.redblobgames.com/pathfinding/a-star/implementation.html

constexpr size_t INVALID_IDX = std::numeric_limits<size_t>::max();

struct Node
{
    uint32_t gCost = -1;
    uint32_t hCost = -1;
    size_t idx = INVALID_IDX;
    size_t parentIdx = INVALID_IDX;

    uint32_t Cost() const { return gCost + hCost; }
};

constexpr int32_t gridWidth = 100;
constexpr int32_t gridHeight = 10;

std::vector<Node> Open;
std::vector<Node> Closed;
std::vector<bool> IsClosed(gridWidth * gridHeight, false); // Prevents havng to check if neighbor is already in closed list
std::vector<size_t> Parent(gridWidth * gridHeight, INVALID_IDX); // filled when a node is closed

size_t startIdx = 5;
size_t endIdx = 399;

// Assuming y is vertical and x horizontal, and (0,0) is topLeft
inline size_t Get1DIdx(int32_t x, int32_t y)
{
    return static_cast<size_t>(gridWidth) * y + x;
}

inline std::pair<int32_t, int32_t> Get2DIdx(size_t idx)
{
    int32_t x = static_cast<int32_t>(idx % gridWidth);
    int32_t y = static_cast<int32_t>(idx / gridWidth);
    return {x, y};
}

// Finds all neighbors for a idx,
// does not clear list, and returns the last valid idx
size_t fillNeighborList(size_t index, std::array<size_t, 8>& neighborList)
{
    auto [x, y] = Get2DIdx(index);

    size_t idx = 0;
    for (int32_t dy = -1; dy <= 1; ++dy)
    {
        for (int32_t dx = -1; dx <= 1; ++dx)
        {
            // Skip center aka ourselves
            if (dx == 0 && dy == 0)
                continue;

            int32_t nx = x + dx;
            int32_t ny = y + dy;

            if (nx < 0 || nx >= gridWidth ||
                ny < 0 || ny >= gridHeight)
                continue;

            neighborList[idx] = Get1DIdx(nx, ny);
            ++idx;
        }
    }
    return idx;
}

uint32_t GetOctileDistance(size_t a, size_t b)
{
    auto [ax, ay] = Get2DIdx(a);
    auto [bx, by] = Get2DIdx(b);

    // Calculate differences
    int32_t dx = std::abs(ax - bx);
    int32_t dy = std::abs(ay - by);

    // Branchless math https://www.geeksforgeeks.org/dsa/a-search-algorithm/ ( 1) Manhattan Distance )
    // Take the minimum of dx and dy as diagonal steps (cost 14)
    // and the remaining steps as straight lines (cost 10)
    return 10 * (dx + dy) + (14 - 2 * 10) * std::min(dx, dy); // (std::min does not suffer from branch prediction :D) 
}

uint32_t GetDistanceBetweenNodes(size_t a, size_t b)
{
    return GetOctileDistance(a, b);
}

void ExploreNeighbors(const Node& current, std::array<size_t, 8>& neighborList)
{
    size_t end = fillNeighborList(current.idx, neighborList);
    for (size_t i = 0; i < end; ++i)
    {
        size_t neighborIdx = neighborList[i];
        if (IsClosed[neighborIdx])
            continue;

        Node nNode;
        nNode.idx = neighborIdx;
        nNode.parentIdx = current.idx;
        nNode.gCost = current.gCost + GetDistanceBetweenNodes(current.idx, neighborIdx);
        nNode.hCost = GetDistanceBetweenNodes(neighborIdx, endIdx);

        // Duplicates are fine for now, PopCheapestOpen discards stale ones
        Open.push_back(nNode);
    }
}

// Removes and returns the cheapest node in Open that is not already closed
Node PopCheapestOpen()
{
    while (!Open.empty())
    {
        auto cheapestOpenit = std::min_element(Open.begin(), Open.end(),
            [](const Node& a, const Node& b) { return a.Cost() < b.Cost(); });

        Node node = *cheapestOpenit;

        //Open.erase(cheapestOpenit);
        // Swap and pop!
        *cheapestOpenit = Open.back();
        Open.pop_back();

        // Stale duplicate of a cell we already finished, skip it
        if (IsClosed[node.idx])
            continue;

        return node;
    }

    throw std::runtime_error("Open list is empty, the end is unreachable");
}

std::deque<size_t> ReconstructPath()
{
    std::deque<size_t> path;
    for (size_t idx = endIdx; idx != INVALID_IDX; idx = Parent[idx])
        path.push_front(idx);

    return path;
}

int main()
{
    std::array<size_t, 8> neighborList{};

    Node startNode;
    startNode.idx = startIdx;
    startNode.gCost = 0;
    // We dont really need to calcualte the h cost of our start node 
    // we dont use it on our first iteration and in following iterations 
    // the explore neighbors will calculate it
    //startNode.hCost = GetDistanceBetweenNodes(startIdx, endIdx);

    Closed.push_back(startNode);
    IsClosed[startIdx] = true;
    Parent[startIdx] = INVALID_IDX;

    while (Closed.back().idx != endIdx)
    {
        ExploreNeighbors(Closed.back(), neighborList);

        Node next = PopCheapestOpen();
        IsClosed[next.idx] = true;
        Parent[next.idx] = next.parentIdx;
        Closed.push_back(next);
    }

    auto path = ReconstructPath();

    // Visualise the final grid state
    Visualizer::PrintGrid(gridWidth, gridHeight, startIdx, endIdx, Open, Closed, path);

    return 0;
}
