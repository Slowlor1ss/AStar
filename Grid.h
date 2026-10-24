#pragma once

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace Pathfinding
{

// TODO: add benefecial path type thats cheaper
enum class TerrainFlags : uint16_t
{
    None  = 0,
    Wall  = 1 << 0,
    Water = 1 << 1,
    Mud   = 1 << 2,
    Lava  = 1 << 3,
};

constexpr TerrainFlags operator|(TerrainFlags a, TerrainFlags b)
{
    return static_cast<TerrainFlags>(static_cast<uint16_t>(a) | static_cast<uint16_t>(b));
}

constexpr TerrainFlags operator&(TerrainFlags a, TerrainFlags b)
{
    return static_cast<TerrainFlags>(static_cast<uint16_t>(a) & static_cast<uint16_t>(b));
}

constexpr TerrainFlags operator~(TerrainFlags a)
{
    return static_cast<TerrainFlags>(~static_cast<uint16_t>(a));
}

constexpr TerrainFlags& operator|=(TerrainFlags& a, TerrainFlags b) { return a = a | b; }
constexpr TerrainFlags& operator&=(TerrainFlags& a, TerrainFlags b) { return a = a & b; }

// True if the cell has at least one of the flags in mask
constexpr bool HasAnyFlag(TerrainFlags cell, TerrainFlags mask)
{
    return (cell & mask) != TerrainFlags::None;
}

// 2D grid of terrain flags
// Dimensions are fixed after construction (anything holding a reference expects this)
class Grid
{
public:
    Grid(int32_t width, int32_t height);

    int32_t GetWidth() const { return m_Width; }
    int32_t GetHeight() const { return m_Height; }
    size_t GetCellCount() const { return m_Terrain.size(); }

    bool IsInBounds(int32_t x, int32_t y) const;
    bool IsValidIdx(size_t idx) const { return idx < m_Terrain.size(); }

    // Assuming y is vertical and x horizontal, and (0,0) is topLeft
    size_t Get1DIdx(int32_t x, int32_t y) const;
    std::pair<int32_t, int32_t> Get2DIdx(size_t idx) const;

    TerrainFlags GetTerrain(int32_t x, int32_t y) const;
    TerrainFlags GetTerrain(size_t idx) const;

    // Replaces the cell's flags
    void SetTerrain(int32_t x, int32_t y, TerrainFlags flags);
    void SetTerrain(size_t idx, TerrainFlags flags);

private:
    int32_t m_Width;
    int32_t m_Height;
    std::vector<TerrainFlags> m_Terrain;
};

}
