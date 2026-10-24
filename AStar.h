#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

// This was a help source I used for ideas
// https://www.redblobgames.com/pathfinding/a-star/implementation.html

inline constexpr size_t INVALID_IDX = std::numeric_limits<size_t>::max();
inline constexpr uint32_t INVALID_COST = std::numeric_limits<uint32_t>::max();

struct Node
{
    uint32_t gCost = INVALID_COST;
    uint32_t hCost = INVALID_COST;
    size_t idx = INVALID_IDX;
    size_t parentIdx = INVALID_IDX;

    uint32_t Cost() const { return gCost + hCost; }
};

// A* on a 2D grid with 8-way movement (octile distance, 10 straight / 14 diagonal).
// Nodes are stored by value in contiguous vectors and referred to by cell index,
// never by pointer, so growing a vector can never leave us with dangling references.
// Storage is owned by the object and reused between searches.
class AStar
{
public:
    AStar(int32_t gridWidth, int32_t gridHeight);

    // Returns false if end is unreachable
    // outPath is cleared first, and start/end inclusive
    bool FindPath(size_t startIdx, size_t endIdx, std::deque<size_t>& outPath);

    // Read-only access for last search (for debug visualisation)
    const std::vector<Node>& GetOpen() const { return m_Open; }
    const std::vector<Node>& GetClosed() const { return m_Closed; }

    int32_t GetGridWidth() const { return m_GridWidth; }
    int32_t GetGridHeight() const { return m_GridHeight; }

private:
    // Assuming y is vertical and x horizontal, and (0,0) is topLeft
    size_t Get1DIdx(int32_t x, int32_t y) const;
    std::pair<int32_t, int32_t> Get2DIdx(size_t idx) const;

    // Finds all neighbors for a idx and stores them in m_NeighborList,
    // does not clear list, and returns the number of valid entries
    size_t FillNeighborList(size_t index);

    uint32_t GetOctileDistance(size_t a, size_t b) const;
    uint32_t GetDistanceBetweenNodes(size_t a, size_t b) const;

    void ExploreNeighbors(const Node& current);

    // Removes and returns the cheapest node in Open that is not already closed,
    // or nothing if no path was found
    std::optional<Node> PopCheapestOpen();

    void ReconstructPath(std::deque<size_t>& outPath) const;

    // Clears all per search states
    void Reset();

    int32_t m_GridWidth;
    int32_t m_GridHeight;

    size_t m_StartIdx = INVALID_IDX;
    size_t m_EndIdx = INVALID_IDX;

    std::vector<Node> m_Open;
    std::vector<Node> m_Closed;
    std::vector<bool> m_IsClosed; // Prevents havng to check if neighbor is already in closed list
    std::vector<size_t> m_Parent; // filled when a node is closed, prevents us from having to look trough the closed list when reconstructing path

    std::array<size_t, 8> m_NeighborList{};
};