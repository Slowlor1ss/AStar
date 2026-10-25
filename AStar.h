#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "Grid.h"
#include "NavProfile.h"

// TODO: maybe split this file up in multiple as its getting a bit messy

// This was a help source I used for ideas
// https://www.redblobgames.com/pathfinding/a-star/implementation.html
namespace Pathfinding
{

inline constexpr size_t INVALID_IDX = std::numeric_limits<size_t>::max();
inline constexpr uint32_t INVALID_COST = std::numeric_limits<uint32_t>::max();
inline constexpr size_t UNLIMITED_EXPANSIONS = std::numeric_limits<size_t>::max();

// Where a search currently is, see AStar::Begin and AStar::Step
enum class SearchStatus : uint8_t
{
    Idle,
    InProgress,
    Found,
    NoPath,
    LimitReached,
    InvalidRequest,
};

struct Node
{
    uint32_t gCost = INVALID_COST;
    uint32_t hCost = INVALID_COST;
    size_t idx = INVALID_IDX;
    size_t parentIdx = INVALID_IDX;

    uint32_t Cost() const { return gCost + hCost; }

    // Spaceship operator! :D
    // Using this also has the benefit of deliting the == operator
    // which I wont implement yet untill needed as i want to be aware of it
    // Weak ordering -> two different nodes can be equivalent 
    // (same f and h) without being equal
    //
    // We prefer the lower h cost in a tie breaker senario
    // this makes up basically prefer to go in the direction of the goal
    // this can makes explore less nodes 
    // and has the added benefit of creating more natural paths
    std::weak_ordering operator<=>(const Node& other) const
    {
        const auto cmp = Cost() <=> other.Cost();
        if (cmp != 0)
            return cmp;
        return hCost <=> other.hCost;
    }

};


// Small wraper for std::vector that acts like a min_heap 
// but with some extended functionality (e.g. a basic clear lol)
// This might be a little bit overkill for our senario, 
// but I find it a lot more explicit then just using a vector with std::push_heap etc.
// this also prevent anyone from breaking the heap
// Push and PopBest are O(log n), looking at the best node is O(1), as expected from a min_heap
class OpenList // purpusefully not made templated as I only want to use it here for our A*
{
public:
    void push(const Node& node)
    {
        m_Nodes.push_back(node);
        std::push_heap(m_Nodes.begin(), m_Nodes.end(), s_Order);
    }
 
    Node pop_best()
    {
        assert(!m_Nodes.empty() && "PopBest on an empty OpenList");
        std::pop_heap(m_Nodes.begin(), m_Nodes.end(), s_Order); // Moves the best node to the back
        const Node best = m_Nodes.back();
        m_Nodes.pop_back();
        return best;
    }
 
    const Node& peek_best() const
    {
        assert(!m_Nodes.empty() && "PeekBest on an empty OpenList");
        return m_Nodes.front();
    }
 
    bool empty() const { return m_Nodes.empty(); }
    size_t size() const { return m_Nodes.size(); }
 
    void clear() { m_Nodes.clear(); }
    void reserve(size_t count) { m_Nodes.reserve(count); }
 
    // Read only view of all open nodes, in heap order 
    // used for debug visualisation
    const std::vector<Node>& get_nodes() const { return m_Nodes; }
 
private:
    // We explicitly use greater as by default 
    // the heap would keep the biggest element at the front using <
    // so we use > to get the cheapest node at the front
    static constexpr std::greater<> s_Order{};
 
    std::vector<Node> m_Nodes;
};

// A* on a 2D grid with 8-way movement (octile distance, 10 straight / 14 diagonal).
// 
// Begin sets a search up, then call Step (e.g. once per frame) until the status is no longer
// InProgress, then fetch the result with GetPath
// Every Step carries on exactly where the previous one stopped
// Calling Begin again stops the search tha was still ongoing
// (The grid's terrain should not change while a search is in progress)
class AStar
{
public:
    // TODO: Optimize for static grid
    // The grid is referenced, not copied: it must outlive this object and keep its dimensions
    // Changing terrain between searches is fine.
    explicit AStar(const Grid& grid);
    AStar(const Grid&&) = delete; // Refuse temporaries, we would be left holding a dangling pointer

    // maxTotalExpansions is the give-up limit for the WHOLE search (-> LimitReached), it keeps
    // unreachable targets from flooding the entire map
    SearchStatus Begin(size_t startIdx, size_t endIdx, const NavProfile& profile = NavProfile{},
                       size_t maxTotalExpansions = UNLIMITED_EXPANSIONS);

    // Expands at most maxExpansions and returns SearchStatus
    // If a search is finished this does nothing
    SearchStatus Step(size_t maxExpansions);

    // Only succeeds when the status is Found
    // outPath is cleared first, and start/end inclusive
    bool GetPath(std::deque<size_t>& outPath) const;

    SearchStatus GetStatus() const { return m_Status; }
    size_t GetExpansionCount() const { return m_ExpansionCount; }

    // Runs whole search with no slicing
    // outPath is cleared first, and start/end inclusive
    bool FindPath(size_t startIdx, size_t endIdx, std::deque<size_t>& outPath,
                  const NavProfile& profile = NavProfile{},
                  size_t maxTotalExpansions = UNLIMITED_EXPANSIONS);

    // Read-only access for last search (for debug visualisation)
    const std::vector<Node>& GetOpen() const { return m_Open.get_nodes(); }
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

    SearchStatus m_Status = SearchStatus::Idle;
    size_t m_ExpansionCount = 0;
    size_t m_ExpansionLimit = UNLIMITED_EXPANSIONS;
    
    // (Min heap of nodes)
    OpenList m_Open;
    std::vector<Node> m_Closed;
    // https://stackoverflow.com/questions/55207770/performance-difference-using-vectorbool-vs-vectoruint8-t
    std::vector<uint8_t> m_IsClosed; // Prevents havng to check if neighbor is already in closed list
    std::vector<size_t> m_Parent; // filled when a node is closed, prevents us from having to look trough the closed list when reconstructing path

    std::array<size_t, 8> m_NeighborList{};
};

}