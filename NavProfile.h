#pragma once

#include "Grid.h"

namespace Pathfinding
{

// The navigation rules of one kind of ai
struct NavProfile
{
    // Terrain this ai cannot enter
    TerrainFlags blockedMask = TerrainFlags::Wall;

    // false: a diagonal step needs both neighbouring cells to be free
    // true: diagonals are allowed as long as the target cell itself is free
    bool allowCornerCutting = false;

    bool IsBlocked(TerrainFlags cell) const { return HasAnyFlag(cell, blockedMask); }
};

}
