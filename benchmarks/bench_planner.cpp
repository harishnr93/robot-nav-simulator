// In-process A* micro-benchmark.
//
// Unlike python/run_benchmark.py, which measures one cold planning call per
// process (what a single CLI invocation sees), this program plans the same map
// many times inside one process and reports the median time per call. That
// removes process start-up noise and shows the steady-state cost of the planner.
//
// Two modes are timed for every map:
//   fresh   planAStar(): new search buffers for every call
//   reused  one AStarPlanner reused for all calls (buffers kept between plans)
//
// Maps: N x N, each cell occupied with probability d (std::mt19937, fixed seeds),
// start (0, 0) and goal (N-1, N-1) cleared. A checksum of costs and expansion
// counts is printed so different builds can be checked for identical results.
//
// Usage: bench_planner [--quick]
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

#include "astar.hpp"
#include "grid_map.hpp"

namespace {

constexpr double kResolution = 0.5;
constexpr int kSeeds = 5;
constexpr long kTargetCellsPerMeasurement = 4'000'000;  // more repeats on small maps

amr::GridMap makeMap(int size, double density, int seed) {
    std::mt19937 rng(static_cast<std::uint32_t>(seed * 1000 + size * 10) +
                     static_cast<std::uint32_t>(density * 100));
    std::bernoulli_distribution occupied(density);
    std::vector<std::uint8_t> cells(static_cast<std::size_t>(size) * static_cast<std::size_t>(size));
    for (auto& cell : cells) {
        cell = occupied(rng) ? amr::GridMap::kOccupied : amr::GridMap::kFree;
    }
    cells.front() = amr::GridMap::kFree;
    cells.back() = amr::GridMap::kFree;
    return amr::GridMap(size, size, kResolution, std::move(cells));
}

template <typename PlanFn>
double medianMicroseconds(int repeats, PlanFn&& plan) {
    std::vector<double> samples;
    samples.reserve(static_cast<std::size_t>(repeats));
    for (int i = 0; i < repeats; ++i) {
        const auto t0 = std::chrono::steady_clock::now();
        plan();
        const auto t1 = std::chrono::steady_clock::now();
        samples.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
    }
    std::nth_element(samples.begin(), samples.begin() + repeats / 2, samples.end());
    return samples[static_cast<std::size_t>(repeats / 2)];
}

double median(std::vector<double> values) {
    std::sort(values.begin(), values.end());
    return values[values.size() / 2];
}

}  // namespace

int main(int argc, char** argv) {
    const bool quick = argc > 1 && std::strcmp(argv[1], "--quick") == 0;
    const std::vector<int> sizes = quick ? std::vector<int>{20, 100}
                                         : std::vector<int>{10, 20, 40, 60, 80, 100, 200, 400};
    const std::vector<double> densities{0.0, 0.1, 0.2, 0.3};

    std::cout << "median time per planning call (us), median over " << kSeeds
              << " maps per row\n";
    std::cout << std::setw(5) << "N" << std::setw(9) << "density" << std::setw(12) << "fresh"
#ifdef AMR_HAS_PLANNER_CLASS
              << std::setw(12) << "reused"
#endif
              << std::setw(12) << "expanded" << '\n';

    std::uint64_t checksum = 0;
    for (const int size : sizes) {
        const long cells = static_cast<long>(size) * size;
        const int repeats = static_cast<int>(std::clamp(kTargetCellsPerMeasurement / cells, 21L, 20001L));
        for (const double density : densities) {
            std::vector<double> fresh_us;
            std::vector<double> reused_us;
            std::vector<double> expanded;
            for (int seed = 0; seed < kSeeds; ++seed) {
                const amr::GridMap map = makeMap(size, density, seed);
                const amr::GridCoord start{0, 0};
                const amr::GridCoord goal{size - 1, size - 1};
                const amr::PlanResult reference = amr::planAStar(map, start, goal);
                checksum = checksum * 1'000'003u + static_cast<std::uint64_t>(reference.path_cost) * 7919u +
                           reference.expanded_nodes;
                for (const amr::GridCoord& c : reference.path) {
                    checksum = checksum * 31u + static_cast<std::uint64_t>(c.x * 65536 + c.y);
                }
                expanded.push_back(static_cast<double>(reference.expanded_nodes));
                fresh_us.push_back(medianMicroseconds(repeats, [&] {
                    volatile std::size_t sink = amr::planAStar(map, start, goal).expanded_nodes;
                    (void)sink;
                }));
#ifdef AMR_HAS_PLANNER_CLASS
                amr::AStarPlanner planner;
                reused_us.push_back(medianMicroseconds(repeats, [&] {
                    volatile std::size_t sink = planner.plan(map, start, goal).expanded_nodes;
                    (void)sink;
                }));
#endif
            }
            std::cout << std::setw(5) << size << std::setw(8) << static_cast<int>(density * 100)
                      << '%' << std::fixed << std::setprecision(2) << std::setw(12)
                      << median(fresh_us)
#ifdef AMR_HAS_PLANNER_CLASS
                      << std::setw(12) << median(reused_us)
#endif
                      << std::setprecision(0) << std::setw(12) << median(expanded) << '\n';
        }
    }
    std::cout << "result checksum: " << checksum << '\n';
    return 0;
}
