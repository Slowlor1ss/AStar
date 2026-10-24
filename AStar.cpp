#include "AStar.h"

#include <algorithm>
#include <cassert>
#include <cstdlib>

namespace Pathfinding
{

AStar::AStar(const Grid& grid)
    : m_pGrid(&grid)
    , m_IsClosed(grid.GetCellCount(), false)
    , m_Parent(grid.GetCellCount(), INVALID_IDX)
{
}


bool AStar::FindPath(size_t startIdx, size_t endIdx, std::deque<size_t>& outPath, const NavProfile& profile)
{
    outPath.clear();

    // TODO: error handling this should never happen if you use the code correctly
    if (!m_pGrid->IsValidIdx(startIdx) || !m_pGrid->IsValidIdx(endIdx))
        return false;
 
    m_Profile = profile;
 
    if (IsBlocked(startIdx) || IsBlocked(endIdx))
        return false;

    Reset();
    m_StartIdx = startIdx;
    m_EndIdx = endIdx;

    Node startNode;
    startNode.idx = m_StartIdx;
    startNode.gCost = 0;
    // We dont really need to calcualte the h cost of our start node 
    // we dont use it on our first iteration and in following iterations 
    // the explore neighbors will calculate it
    //startNode.hCost = GetDistanceBetweenNodes(startIdx, endIdx);

    m_Closed.push_back(startNode);
    m_IsClosed[m_StartIdx] = true;
    m_Parent[m_StartIdx] = INVALID_IDX;

    while (m_Closed.back().idx != m_EndIdx)
    {
        ExploreNeighbors(m_Closed.back());

        std::optional<Node> next = PopCheapestOpen();
        if (!next)
            return false; // End is unreachable

        m_IsClosed[next->idx] = true;
        m_Parent[next->idx] = next->parentIdx;
        m_Closed.push_back(*next);
    }

    ReconstructPath(outPath);
    return true;
}

void AStar::Reset()
{
    m_Open.clear();
    m_Closed.clear();
    std::fill(m_IsClosed.begin(), m_IsClosed.end(), false);
    std::fill(m_Parent.begin(), m_Parent.end(), INVALID_IDX);
}

bool AStar::IsBlocked(int32_t x, int32_t y) const
{
    return m_Profile.IsBlocked(m_pGrid->GetTerrain(x, y));
}
 
bool AStar::IsBlocked(size_t idx) const
{
    return m_Profile.IsBlocked(m_pGrid->GetTerrain(idx));
}

size_t AStar::FillNeighborList(size_t index)
{
    auto [x, y] = m_pGrid->Get2DIdx(index);

    size_t count = 0;
    for (int32_t dy = -1; dy <= 1; ++dy)
    {
        for (int32_t dx = -1; dx <= 1; ++dx)
        {
            // Skip center aka ourselves
            if (dx == 0 && dy == 0)
                continue;

            int32_t nx = x + dx;
            int32_t ny = y + dy;

            if (!m_pGrid->IsInBounds(nx, ny))
                continue;
 
            if (IsBlocked(nx, ny))
                continue;

            // Handle diagonal, if allowed we skip the blocked check, 
            // if not make sure common adjacent cells are free
            const bool isDiagonal = dx != 0 && dy != 0;
            if (isDiagonal && !m_Profile.allowCornerCutting &&
                (IsBlocked(nx, y) || IsBlocked(x, ny)))
                continue;


            m_NeighborList[count] = m_pGrid->Get1DIdx(nx, ny);
            ++count;
        }
    }
    return count;
}

uint32_t AStar::GetOctileDistance(size_t a, size_t b) const
{
    auto [ax, ay] = m_pGrid->Get2DIdx(a);
    auto [bx, by] = m_pGrid->Get2DIdx(b);

    // Calculate differences
    int32_t dx = std::abs(ax - bx);
    int32_t dy = std::abs(ay - by);

    // Branchless math https://www.geeksforgeeks.org/dsa/a-search-algorithm/ ( 1) Manhattan Distance )
    // Take the minimum of dx and dy as diagonal steps (cost 14)
    // and the remaining steps as straight lines (cost 10)
    return 10 * (dx + dy) + (14 - 2 * 10) * std::min(dx, dy); // (std::min does not suffer from branch prediction :D) 
}

// I made this a little wrapper just in case we want to swap to a different method
uint32_t AStar::GetDistanceBetweenNodes(size_t a, size_t b) const
{
    return GetOctileDistance(a, b);
}

uint32_t AStar::GetStepCost(size_t from, size_t to) const
{
    // Octile distance between two adjacent cells is exactly the base step: 10 or 14
    const uint32_t baseStep = GetDistanceBetweenNodes(from, to);
    const uint32_t costPercent = m_Profile.GetCellCostPercent(m_pGrid->GetTerrain(to));
    return baseStep * costPercent / COST_PERCENT_NORMAL;
}

void AStar::ExploreNeighbors(const Node& current)
{
    size_t end = FillNeighborList(current.idx);
    for (size_t i = 0; i < end; ++i)
    {
        size_t neighborIdx = m_NeighborList[i];
        if (m_IsClosed[neighborIdx])
            continue;

        Node nNode;
        nNode.idx = neighborIdx;
        nNode.parentIdx = current.idx;
                nNode.gCost = current.gCost + GetStepCost(current.idx, neighborIdx);
        nNode.hCost = GetDistanceBetweenNodes(neighborIdx, m_EndIdx);

        // Duplicates are fine for now, PopCheapestOpen discards stale ones
        m_Open.push_back(nNode);
    }
}

std::optional<Node> AStar::PopCheapestOpen()
{
    while (!m_Open.empty())
    {
        auto cheapestOpenit = std::min_element(m_Open.begin(), m_Open.end(),
            [](const Node& a, const Node& b) { return a.Cost() < b.Cost(); });

        Node node = *cheapestOpenit;

        //m_Open.erase(cheapestOpenit);
        // Swap and pop!
        *cheapestOpenit = m_Open.back();
        m_Open.pop_back();

        // Stale duplicate of a cell we already finished, skip it
        if (m_IsClosed[node.idx])
            continue;

        return node;
    }

    return std::nullopt;
}

void AStar::ReconstructPath(std::deque<size_t>& outPath) const
{
    for (size_t idx = m_EndIdx; idx != INVALID_IDX; idx = m_Parent[idx])
        outPath.push_front(idx);
}

}