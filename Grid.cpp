#include "Grid.h"

#include <cassert>

namespace Pathfinding
{

Grid::Grid(int32_t width, int32_t height)
    : m_Width(width)
    , m_Height(height)
    , m_Terrain(static_cast<size_t>(width) * height, TerrainFlags::None)
{
    assert(width > 0 && height > 0 && "Grid dimensions must be positive");
}

bool Grid::IsInBounds(int32_t x, int32_t y) const
{
    return x >= 0 && x < m_Width &&
           y >= 0 && y < m_Height;
}

size_t Grid::Get1DIdx(int32_t x, int32_t y) const
{
    return static_cast<size_t>(m_Width) * y + x;
}

std::pair<int32_t, int32_t> Grid::Get2DIdx(size_t idx) const
{
    int32_t x = static_cast<int32_t>(idx % m_Width);
    int32_t y = static_cast<int32_t>(idx / m_Width);
    return {x, y};
}

// TODO: make own asserts that dont do anything in non debug mode
TerrainFlags Grid::GetTerrain(int32_t x, int32_t y) const
{
    assert(IsInBounds(x, y));
    return m_Terrain[Get1DIdx(x, y)];
}

TerrainFlags Grid::GetTerrain(size_t idx) const
{
    assert(IsValidIdx(idx));
    return m_Terrain[idx];
}

void Grid::SetTerrain(int32_t x, int32_t y, TerrainFlags flags)
{
    assert(IsInBounds(x, y));
    m_Terrain[Get1DIdx(x, y)] = flags;
}

void Grid::SetTerrain(size_t idx, TerrainFlags flags)
{
    assert(IsValidIdx(idx));
    m_Terrain[idx] = flags;
}

}
