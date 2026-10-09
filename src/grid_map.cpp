#include "grid_map.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace amr {

namespace {

std::size_t checkedCellCount(int width, int height, double resolution) {
    if (width <= 0 || height <= 0) {
        throw std::invalid_argument("map width and height must be positive");
    }
    if (width > GridMap::kMaxDimension || height > GridMap::kMaxDimension) {
        throw std::invalid_argument("map dimensions exceed the supported maximum");
    }
    if (!std::isfinite(resolution) || resolution <= 0.0) {
        throw std::invalid_argument("map resolution must be a positive finite number");
    }
    return static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
}

std::string trim(const std::string& text) {
    const char* whitespace = " \t\r\n";
    const std::size_t first = text.find_first_not_of(whitespace);
    if (first == std::string::npos) {
        return {};
    }
    const std::size_t last = text.find_last_not_of(whitespace);
    return text.substr(first, last - first + 1);
}

MapLoadResult failure(std::string message) {
    MapLoadResult result;
    result.error = std::move(message);
    return result;
}

}  // namespace

GridMap::GridMap(int width, int height, double resolution)
    : GridMap(width, height, resolution,
              std::vector<std::uint8_t>(checkedCellCount(width, height, resolution), kFree)) {}

GridMap::GridMap(int width, int height, double resolution, std::vector<std::uint8_t> cells)
    : width_(width), height_(height), resolution_(resolution), cells_(std::move(cells)) {
    if (cells_.size() != checkedCellCount(width, height, resolution)) {
        throw std::invalid_argument("cell count does not match width * height");
    }
    for (const std::uint8_t cell : cells_) {
        if (cell != kFree && cell != kOccupied) {
            throw std::invalid_argument("cell values must be 0 (free) or 1 (occupied)");
        }
    }
}

bool GridMap::inBounds(GridCoord c) const {
    return c.x >= 0 && c.y >= 0 && c.x < width_ && c.y < height_;
}

bool GridMap::isOccupied(GridCoord c) const {
    return !inBounds(c) || cells_[index(c)] == kOccupied;
}

bool GridMap::isFree(GridCoord c) const {
    return !isOccupied(c);
}

void GridMap::setOccupied(GridCoord c, bool occupied) {
    cells_[index(c)] = occupied ? kOccupied : kFree;
}

std::size_t GridMap::index(GridCoord c) const {
    return static_cast<std::size_t>(c.y) * static_cast<std::size_t>(width_) +
           static_cast<std::size_t>(c.x);
}

GridCoord GridMap::coordFromIndex(std::size_t index) const {
    const auto w = static_cast<std::size_t>(width_);
    return GridCoord{static_cast<int>(index % w), static_cast<int>(index / w)};
}

WorldPoint GridMap::gridToWorld(GridCoord c) const {
    return WorldPoint{(c.x + 0.5) * resolution_, (c.y + 0.5) * resolution_};
}

MapLoadResult parseGridMapCsv(std::istream& input, double resolution) {
    if (!std::isfinite(resolution) || resolution <= 0.0) {
        return failure("map resolution must be a positive finite number");
    }

    std::vector<std::uint8_t> cells;
    int width = -1;
    int height = 0;
    int line_number = 0;
    std::string line;

    while (std::getline(input, line)) {
        ++line_number;
        const std::string content = trim(line);
        if (content.empty() || content.front() == '#') {
            continue;
        }

        int columns = 0;
        std::size_t token_start = 0;
        while (true) {
            const std::size_t comma = content.find(',', token_start);
            const std::size_t token_length =
                comma == std::string::npos ? std::string::npos : comma - token_start;
            const std::string token = trim(content.substr(token_start, token_length));
            ++columns;

            std::ostringstream where;
            where << "line " << line_number << ", column " << columns;
            if (token == "0") {
                cells.push_back(GridMap::kFree);
            } else if (token == "1") {
                cells.push_back(GridMap::kOccupied);
            } else if (token.empty()) {
                return failure(where.str() + ": empty cell (expected 0 or 1)");
            } else {
                return failure(where.str() + ": unsupported cell value '" + token +
                               "' (expected 0 or 1)");
            }
            if (columns > GridMap::kMaxDimension) {
                return failure(where.str() + ": row is wider than the supported maximum");
            }
            if (comma == std::string::npos) {
                break;
            }
            token_start = comma + 1;
        }

        if (width < 0) {
            width = columns;
        } else if (columns != width) {
            std::ostringstream message;
            message << "line " << line_number << " has " << columns << " cells, expected "
                    << width << " (the grid must be rectangular)";
            return failure(message.str());
        }
        ++height;
        if (height > GridMap::kMaxDimension) {
            return failure("map has more rows than the supported maximum");
        }
    }

    if (input.bad()) {
        return failure("I/O error while reading map data");
    }
    if (height == 0) {
        return failure("map contains no grid rows");
    }

    MapLoadResult result;
    result.map = GridMap(width, height, resolution, std::move(cells));
    result.ok = true;
    return result;
}

MapLoadResult loadGridMapCsv(const std::filesystem::path& file, double resolution) {
    const std::string prefix = "map file '" + file.string() + "': ";
    std::error_code ec;
    if (!std::filesystem::exists(file, ec)) {
        return failure(prefix + "does not exist");
    }
    if (!std::filesystem::is_regular_file(file, ec)) {
        return failure(prefix + "is not a regular file");
    }
    const auto size = std::filesystem::file_size(file, ec);
    if (ec) {
        return failure(prefix + "cannot determine file size: " + ec.message());
    }
    if (size == 0) {
        return failure(prefix + "is empty");
    }

    std::ifstream input(file);
    if (!input) {
        return failure(prefix + "cannot be opened for reading");
    }
    MapLoadResult result = parseGridMapCsv(input, resolution);
    if (!result.ok) {
        result.error = prefix + result.error;
    }
    return result;
}

}  // namespace amr
