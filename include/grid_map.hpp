// Occupancy-grid representation and CSV map loading.
//
// Map file format: one grid row per line, cells separated by commas,
// each cell either 0 (free) or 1 (occupied). The first line is row y = 0
// (the top of the map). Blank lines and lines starting with '#' are ignored.
#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <istream>
#include <string>
#include <vector>

#include "types.hpp"

namespace amr {

class GridMap {
public:
    static constexpr std::uint8_t kFree = 0;
    static constexpr std::uint8_t kOccupied = 1;
    /// Upper bound on width and height; protects against absurd inputs.
    static constexpr int kMaxDimension = 10000;

    /// An empty 0 x 0 map. Every coordinate is out of bounds.
    GridMap() = default;

    /// All-free map. Throws std::invalid_argument on invalid dimensions.
    GridMap(int width, int height, double resolution);

    /// Map from row-major cells (index = y * width + x).
    /// Throws std::invalid_argument on invalid dimensions or cell values.
    GridMap(int width, int height, double resolution, std::vector<std::uint8_t> cells);

    int width() const { return width_; }
    int height() const { return height_; }
    double resolution() const { return resolution_; }
    std::size_t cellCount() const { return cells_.size(); }

    bool inBounds(GridCoord c) const;
    /// Out-of-bounds cells are reported as occupied (conservative).
    bool isOccupied(GridCoord c) const;
    /// True only for in-bounds, unoccupied cells.
    bool isFree(GridCoord c) const;
    /// Precondition: inBounds(c).
    void setOccupied(GridCoord c, bool occupied);

    /// Row-major index. Precondition: inBounds(c).
    std::size_t index(GridCoord c) const;
    /// Inverse of index(). Precondition: index < cellCount().
    GridCoord coordFromIndex(std::size_t index) const;

    /// Center of the cell in meters.
    WorldPoint gridToWorld(GridCoord c) const;

private:
    int width_ = 0;
    int height_ = 0;
    double resolution_ = 1.0;
    std::vector<std::uint8_t> cells_;
};

/// Outcome of loading a map. When ok is false, error explains why.
struct MapLoadResult {
    bool ok = false;
    GridMap map;
    std::string error;
};

/// Parse a map from any stream (used by the file loader and by tests).
MapLoadResult parseGridMapCsv(std::istream& input, double resolution);

/// Load and validate a map file. Never throws for bad input; returns an error instead.
MapLoadResult loadGridMapCsv(const std::filesystem::path& file, double resolution);

}  // namespace amr
