#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "Grid.h"
#include "NavProfile.h"

// This was a help source I used for ideas
// https://www.redblobgames.com/pathfinding/a-star/implementation.html
namespace Pathfinding
{

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
    // TODO: Optimize for static grid
    // The grid is referenced, not copied: it must outlive this object and keep its dimensions
    // Changing terrain between searches is fine.
    explicit AStar(const Grid& grid);
    AStar(const Grid&&) = delete; // Refuse temporaries, we would be left holding a dangling pointer

    // Returns false if end is unreachable
    // outPath is cleared first, and start/end inclusive
    bool FindPath(size_t startIdx, size_t endIdx, std::deque<size_t>& outPath,
                  const NavProfile& profile = NavProfile{});

    // Read-only access for last search (for debug visualisation)
    const std::vector<Node>& GetOpen() const { return m_Open; }
    const std::vector<Node>& GetClosed() const { return m_Closed; }

private:
    bool IsBlocked(int32_t x, int32_t y) const;
    bool IsBlocked(size_t idx) const;

    // Finds all neighbors for a idx and stores them in m_NeighborList,
    // does not clear list, and returns the number of valid entries
    size_t FillNeighborList(size_t index);

    uint32_t GetOctileDistance(size_t a, size_t b) const;
    uint32_t GetDistanceBetweenNodes(size_t a, size_t b) const;
    uint32_t GetStepCost(size_t from, size_t to) const;

    void ExploreNeighbors(const Node& current);

    // Removes and returns the cheapest node in Open that is not already closed,
    // or nothing if no path was found
    std::optional<Node> PopCheapestOpen();

    void ReconstructPath(std::deque<size_t>& outPath) const;

    // Clears all per search states
    void Reset();

    const Grid* m_pGrid;
    NavProfile m_Profile; // copy of the rules for the search in progress

    size_t m_StartIdx = INVALID_IDX;
    size_t m_EndIdx = INVALID_IDX;

    std::vector<Node> m_Open;
    std::vector<Node> m_Closed;
    // https://stackoverflow.com/questions/55207770/performance-difference-using-vectorbool-vs-vectoruint8-t
    std::vector<uint8_t> m_IsClosed; // Prevents havng to check if neighbor is already in closed list
    std::vector<size_t> m_Parent; // filled when a node is closed, prevents us from having to look trough the closed list when reconstructing path

    std::array<size_t, 8> m_NeighborList{};
};

}