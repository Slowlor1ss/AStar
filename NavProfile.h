#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cassert>
#include <cstddef>
#include <cstdint>

#include "Grid.h"

namespace Pathfinding
{

// TODO-Maybe: cell cost blending
// Cost of entering a cell is a percentage of the normal cost
// Using ints everywhere, to avaoid floating point math
inline constexpr uint16_t COST_PERCENT_NORMAL = 100;

// The navigation rules for one kind of ai
struct NavProfile
{
    // Terrain this ai cannot enter
    TerrainFlags blockedMask = TerrainFlags::Wall;

    // false: diagonals needs both neighbouring cells to be free
    // true:  diagonals are allowed if the target cell itself is free
    bool allowCornerCutting = false;

    constexpr NavProfile() { m_TerrainCostPercent.fill(COST_PERCENT_NORMAL); }

    bool IsBlocked(TerrainFlags cell) const { return HasAnyFlag(cell, blockedMask); }

    // Sets what entering any of the given terrain types costs this ai
    // !!! Must be >= COST_PERCENT_NORMAL otherwise we could overestmate and end up with a bad path
    void SetTerrainCost(TerrainFlags terrain, uint16_t percent)
    {
        assert(percent >= COST_PERCENT_NORMAL && "Terrain cheaper than normal breaks the heuristic");
        percent = std::max(percent, COST_PERCENT_NORMAL);

        TerrainType_t bits = static_cast<TerrainType_t>(terrain);
        while (bits != 0)
        {
            m_TerrainCostPercent[static_cast<size_t>(std::countr_zero(bits))] = percent;
            bits &= static_cast<TerrainType_t>(bits - 1); // Clear the lowest set bit
        }
    }

    // Cost of entering a cell with these flags
    // A cell with multiple flags costs = most expensive one
    uint16_t GetCellCostPercent(TerrainFlags cell) const
    {
        TerrainType_t bits = static_cast<TerrainType_t>(cell);
        uint16_t cost = COST_PERCENT_NORMAL;
        while (bits != 0)
        {
            cost = std::max(cost, m_TerrainCostPercent[static_cast<size_t>(std::countr_zero(bits))]);
            bits &= static_cast<TerrainType_t>(bits - 1); // Clear the lowest set bit
        }
        return cost;
    }

private:
    // Indexed by the bit position of the terrain flag, 
    // private so the >= normal rule can't be bypassed :)
    std::array<uint16_t, TERRAIN_TYPE_COUNT> m_TerrainCostPercent{};
};

}