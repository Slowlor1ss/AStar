// Plain self-checking tests for AStar, no framework needed.
// CHECK always runs (unlike assert it is not compiled out with NDEBUG).
//
//   g++ -std=c++23 -O1 -Wall -Wextra AStarTests.cpp AStar.cpp Grid.cpp -o astar_tests && ./astar_tests

#include <cstdlib>
#include <deque>
#include <iostream>
#include <queue>
#include <random>
#include <vector>

#include "AStar.h"

using namespace Pathfinding;

#define CHECK(cond)                                                                  \
    do {                                                                             \
        if (!(cond)) {                                                               \
            std::cerr << __FILE__ << ":" << __LINE__ << "  CHECK failed: " #cond "\n"; \
            std::exit(1);                                                            \
        }                                                                            \
    } while (0)

namespace
{

// ---------- helpers ----------

// Independent reference implementation: plain Dijkstra with the same movement rules.
// Returns the cheapest cost from s to e, or UINT32_MAX if unreachable.
uint32_t DijkstraCost(const Grid& g, const NavProfile& p, size_t s, size_t e)
{
    std::vector<uint32_t> dist(g.GetCellCount(), UINT32_MAX);
    using Item = std::pair<uint32_t, size_t>;
    std::priority_queue<Item, std::vector<Item>, std::greater<>> pq;
    dist[s] = 0;
    pq.push({0, s});
    while (!pq.empty())
    {
        auto [d, i] = pq.top();
        pq.pop();
        if (d > dist[i])
            continue;
        auto [x, y] = g.Get2DIdx(i);
        for (int dy = -1; dy <= 1; ++dy)
        {
            for (int dx = -1; dx <= 1; ++dx)
            {
                if (dx == 0 && dy == 0)
                    continue;

                int nx = x + dx, ny = y + dy;
                if (!g.IsInBounds(nx, ny) || p.IsBlocked(g.GetTerrain(nx, ny)))
                    continue;

                const bool diag = dx != 0 && dy != 0;
                if (diag && !p.allowCornerCutting 
                    && (p.IsBlocked(g.GetTerrain(nx, y)) 
                    || p.IsBlocked(g.GetTerrain(x, ny))))
                {
                    continue;
                }

                const uint32_t base = diag ? 14 : 10;
                const uint32_t c = base * p.GetCellCostPercent(g.GetTerrain(nx, ny)) / 100;
                const size_t ni = g.Get1DIdx(nx, ny);
                if (d + c < dist[ni])
                {
                    dist[ni] = d + c;
                    pq.push({d + c, ni});
                }
            }
        }
    }
    return dist[e];
}

struct Scenario
{
    Grid grid;
    NavProfile profile;
    size_t start = 0;
    size_t end = 0;
};

// Random level with walls, mud, water and mixed cells, random costs and corner rules.
// Start and end are always walkable cells.
Scenario MakeRandomScenario(std::mt19937& rng)
{
    const int w = 4 + static_cast<int>(rng() % 16);
    const int h = 4 + static_cast<int>(rng() % 16);
    Scenario sc{Grid(w, h), NavProfile{}, 0, 0};

    for (size_t i = 0; i < sc.grid.GetCellCount(); ++i)
    {
        const unsigned r = static_cast<unsigned>(rng() % 100);
        if (r < 22)      sc.grid.SetTerrain(i, TerrainFlags::Wall);
        else if (r < 38) sc.grid.SetTerrain(i, TerrainFlags::Mud);
        else if (r < 52) sc.grid.SetTerrain(i, TerrainFlags::Water);
        else if (r < 58) sc.grid.SetTerrain(i, TerrainFlags::Water | TerrainFlags::Mud);
        // Rest is just none
    }
    sc.profile.SetTerrainCost(TerrainFlags::Mud, static_cast<uint16_t>(100 + rng() % 400));
    sc.profile.SetTerrainCost(TerrainFlags::Water, static_cast<uint16_t>(100 + rng() % 400));
    sc.profile.allowCornerCutting = rng() % 2 == 0;

    auto pickWalkable = [&]
    {
        size_t idx;
        do { idx = rng() % sc.grid.GetCellCount(); } while (sc.profile.IsBlocked(sc.grid.GetTerrain(idx)));
        return idx;
    };
    sc.start = pickWalkable();
    sc.end = pickWalkable();
    return sc;
}

// Drives a search to completion in slices of `slice` expansions
SearchStatus RunSliced(AStar& a, size_t slice)
{
    SearchStatus status = a.GetStatus();
    while (status == SearchStatus::InProgress)
        status = a.Step(slice);
    return status;
}

// ---------- tests ----------

// FindPath must be optimal: same cost as Dijkstra, and agree on reachability
void TestMatchesDijkstra()
{
    std::mt19937 rng(12345);
    int found = 0, notFound = 0;
    for (int it = 0; it < 2000; ++it)
    {
        Scenario sc = MakeRandomScenario(rng);
        AStar a(sc.grid);
        std::deque<size_t> path;
        const bool ok = a.FindPath(sc.start, sc.end, path, sc.profile);
        const uint32_t ref = DijkstraCost(sc.grid, sc.profile, sc.start, sc.end);

        CHECK(ok == (ref != UINT32_MAX));
        if (ok)
        {
            ++found;
            CHECK(a.GetClosed().back().gCost == ref);
            CHECK(path.front() == sc.start && path.back() == sc.end);
            for (size_t idx : path)
                CHECK(!sc.profile.IsBlocked(sc.grid.GetTerrain(idx)));
        }
        else
        {
            ++notFound;
            CHECK(path.empty());
        }
    }
    CHECK(found > 500 && notFound > 5); // make sure the random levels actually cover both outcomes
}

// Spreading a search over many Step calls must give exactly the same result as running it in one go
void TestSlicedEqualsFindPath()
{
    std::mt19937 rng(777);
    for (int it = 0; it < 1000; ++it)
    {
        Scenario sc = MakeRandomScenario(rng);

        AStar whole(sc.grid);
        std::deque<size_t> wholePath;
        const bool ok = whole.FindPath(sc.start, sc.end, wholePath, sc.profile);

        AStar sliced(sc.grid);
        CHECK(sliced.Begin(sc.start, sc.end, sc.profile) == SearchStatus::InProgress);
        const size_t slice = 1 + rng() % 40;
        const SearchStatus status = RunSliced(sliced, slice);

        CHECK((status == SearchStatus::Found) == ok);
        CHECK(status == (ok ? SearchStatus::Found : SearchStatus::NoPath));
        CHECK(sliced.GetExpansionCount() == whole.GetExpansionCount());

        std::deque<size_t> slicedPath;
        CHECK(sliced.GetPath(slicedPath) == ok);
        CHECK(slicedPath == wholePath);
    }
}

// The total limit: exactly enough expansions finds the path, one fewer gives up
void TestExpansionLimit()
{
    std::mt19937 rng(4242);
    int boundaryCases = 0;
    for (int it = 0; it < 500; ++it)
    {
        Scenario sc = MakeRandomScenario(rng);

        AStar reference(sc.grid);
        std::deque<size_t> path;
        if (!reference.FindPath(sc.start, sc.end, path, sc.profile))
            continue;
        const size_t needed = reference.GetExpansionCount();
        if (needed == 0)
            continue; // start == end

        AStar a(sc.grid);
        CHECK(a.Begin(sc.start, sc.end, sc.profile, needed) == SearchStatus::InProgress);
        CHECK(RunSliced(a, 7) == SearchStatus::Found);

        CHECK(a.Begin(sc.start, sc.end, sc.profile, needed - 1) == SearchStatus::InProgress);
        CHECK(RunSliced(a, 7) == SearchStatus::LimitReached);
        CHECK(a.GetExpansionCount() == needed - 1);

        // FindPath honours the limit too, and never hands out a path when it gave up
        CHECK(!a.FindPath(sc.start, sc.end, path, sc.profile, needed - 1));
        CHECK(path.empty());
        ++boundaryCases;
    }
    CHECK(boundaryCases > 200);
}

void TestStatusAndEdgeCases()
{
    Grid g(10, 10);
    g.SetTerrain(5, 5, TerrainFlags::Wall);
    AStar a(g);
    std::deque<size_t> path{1, 2, 3};

    CHECK(a.GetStatus() == SearchStatus::Idle);
    CHECK(a.Step(100) == SearchStatus::Idle);          // nothing started, nothing happens
    CHECK(!a.GetPath(path) && path.empty());           // and no path, outPath is cleared

    CHECK(a.Begin(0, 1000) == SearchStatus::InvalidRequest);
    CHECK(a.Begin(1000, 0) == SearchStatus::InvalidRequest);
    CHECK(a.Begin(g.Get1DIdx(5, 5), 0) == SearchStatus::NoPath); // start is a wall
    CHECK(a.Begin(0, g.Get1DIdx(5, 5)) == SearchStatus::NoPath); // end is a wall
    CHECK(a.GetClosed().empty() && a.GetOpen().empty());         // early outs leave no stale state

    // start == end is found without expanding anything
    CHECK(a.Begin(7, 7) == SearchStatus::InProgress);
    CHECK(a.Step(0) == SearchStatus::Found);
    CHECK(a.GetExpansionCount() == 0);
    CHECK(a.GetPath(path) && path.size() == 1 && path[0] == 7);

    // A budget of 0 does no work, a budget of 1 does exactly one expansion
    CHECK(a.Begin(0, g.Get1DIdx(9, 9)) == SearchStatus::InProgress);
    CHECK(a.Step(0) == SearchStatus::InProgress && a.GetExpansionCount() == 0);
    CHECK(a.Step(1) == SearchStatus::InProgress && a.GetExpansionCount() == 1);
    CHECK(!a.GetPath(path));                                      // not finished yet

    // A finished search is sticky
    CHECK(RunSliced(a, 5) == SearchStatus::Found);
    const size_t expansions = a.GetExpansionCount();
    CHECK(a.Step(100) == SearchStatus::Found && a.GetExpansionCount() == expansions);
    CHECK(a.GetPath(path) && path.front() == 0 && path.back() == g.Get1DIdx(9, 9));
}

// Starting a new search while one is still running must not leak anything from the old one
void TestAbandoningASearch()
{
    std::mt19937 rng(99);
    for (int it = 0; it < 300; ++it)
    {
        Scenario sc = MakeRandomScenario(rng);
        const size_t otherStart = [&] {
            size_t i;
            do { i = rng() % sc.grid.GetCellCount(); } while (sc.profile.IsBlocked(sc.grid.GetTerrain(i)));
            return i;
        }();

        AStar fresh(sc.grid);
        std::deque<size_t> expected;
        const bool ok = fresh.FindPath(otherStart, sc.end, expected, sc.profile);

        AStar a(sc.grid);
        a.Begin(sc.start, sc.end, sc.profile);
        a.Step(1 + rng() % 5);                       // leave the first search half done
        a.Begin(otherStart, sc.end, sc.profile);     // ... and abandon it
        const SearchStatus status = RunSliced(a, 3);

        CHECK((status == SearchStatus::Found) == ok);
        std::deque<size_t> got;
        CHECK(a.GetPath(got) == ok);
        CHECK(got == expected);
        CHECK(a.GetExpansionCount() == fresh.GetExpansionCount());
    }
}

// The reason the limit exists: a target that cannot be reached floods the whole map
void TestUnreachableTargetIsBounded()
{
    Grid g(300, 300);
    const int gx = 150, gy = 150;
    for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx)
            if (dx != 0 || dy != 0)
                g.SetTerrain(gx + dx, gy + dy, TerrainFlags::Wall); // goal sealed in by a ring of walls
    AStar a(g);
    std::deque<size_t> path;
    const size_t goal = g.Get1DIdx(gx, gy);

    CHECK(!a.FindPath(0, goal, path));
    CHECK(a.GetStatus() == SearchStatus::NoPath);
    CHECK(a.GetExpansionCount() > 85000);            // had to explore (almost) the entire map to find out

    CHECK(!a.FindPath(0, goal, path, NavProfile{}, 1000));
    CHECK(a.GetStatus() == SearchStatus::LimitReached);
    CHECK(a.GetExpansionCount() == 1000);            // gave up after exactly the allowed work
}

} // namespace

int main()
{
    TestMatchesDijkstra();
    TestSlicedEqualsFindPath();
    TestExpansionLimit();
    TestStatusAndEdgeCases();
    TestAbandoningASearch();
    TestUnreachableTargetIsBounded();
    std::cout << "all AStar tests passed\n";
    return 0;
}
