// Tests for occupancy-grid representation, coordinate conversion and map parsing.
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include "astar.hpp"
#include "grid_map.hpp"
#include "test_framework.hpp"

using amr::GridCoord;
using amr::GridMap;

namespace {

constexpr double kResolution = 0.5;
constexpr double kTolerance = 1e-9;

amr::MapLoadResult parse(const std::string& text, double resolution = kResolution) {
    std::istringstream input(text);
    return amr::parseGridMapCsv(input, resolution);
}

/// Writes text to a uniquely named temporary file and removes it on destruction.
class TempFile {
public:
    TempFile(const std::string& name, const std::string& content)
        : path_(std::filesystem::temp_directory_path() / ("amr_test_" + name)) {
        std::ofstream out(path_, std::ios::binary);
        out << content;
    }
    ~TempFile() {
        std::error_code ec;
        std::filesystem::remove(path_, ec);
    }
    TempFile(const TempFile&) = delete;
    TempFile& operator=(const TempFile&) = delete;
    const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
};

void testParsesValidMapRowMajor() {
    const auto result = parse("0,0,1\n0,1,0\n");
    REQUIRE(result.ok);
    CHECK_EQ(result.map.width(), 3);
    CHECK_EQ(result.map.height(), 2);
    CHECK_EQ(result.map.cellCount(), static_cast<std::size_t>(6));
    CHECK(result.map.isOccupied({2, 0}));  // first line is row y = 0
    CHECK(result.map.isOccupied({1, 1}));
    CHECK(result.map.isFree({0, 0}));
    CHECK(result.map.isFree({2, 1}));
    CHECK_EQ(result.map.index({2, 1}), static_cast<std::size_t>(5));
    CHECK_EQ(result.map.coordFromIndex(5), (GridCoord{2, 1}));
}

void testToleratesWhitespaceCommentsAndCrlf() {
    const auto result = parse("# comment line\r\n 0 , 1 \r\n\r\n1,0\r\n# trailing comment\n");
    REQUIRE(result.ok);
    CHECK_EQ(result.map.width(), 2);
    CHECK_EQ(result.map.height(), 2);
    CHECK(result.map.isOccupied({1, 0}));
    CHECK(result.map.isOccupied({0, 1}));
}

void testCoordinateConversion() {
    const GridMap map(20, 20, kResolution);
    const auto origin = map.gridToWorld({0, 0});
    CHECK_NEAR(origin.x, 0.25, kTolerance);
    CHECK_NEAR(origin.y, 0.25, kTolerance);
    const auto cell = map.gridToWorld({3, 1});
    CHECK_NEAR(cell.x, 1.75, kTolerance);
    CHECK_NEAR(cell.y, 0.75, kTolerance);
    const auto corner = map.gridToWorld({19, 19});
    CHECK_NEAR(corner.x, 9.75, kTolerance);
    CHECK_NEAR(corner.y, 9.75, kTolerance);
}

void testWorldToGrid() {
    const GridMap map(20, 10, kResolution);
    CHECK_EQ(map.worldToGrid({0.25, 0.25}), (GridCoord{0, 0}));
    CHECK_EQ(map.worldToGrid({1.75, 0.75}), (GridCoord{3, 1}));
    CHECK_EQ(map.worldToGrid({0.0, 0.0}), (GridCoord{0, 0}));
    CHECK_EQ(map.worldToGrid({0.5, 0.49}), (GridCoord{1, 0}));  // boundary -> larger index
    CHECK_EQ(map.worldToGrid({9.99, 4.99}), (GridCoord{19, 9}));
    CHECK(!map.inBounds(map.worldToGrid({10.0, 1.0})));   // right edge is outside
    CHECK(!map.inBounds(map.worldToGrid({-0.01, 1.0})));
    CHECK(!map.inBounds(map.worldToGrid({1.0, 5.0})));
    CHECK(!map.inBounds(map.worldToGrid({std::nan(""), 1.0})));
    CHECK(!map.inBounds(map.worldToGrid({1e300, 1.0})));
    // Round trip: every cell center maps back to its own cell.
    for (int y = 0; y < map.height(); ++y) {
        for (int x = 0; x < map.width(); ++x) {
            CHECK_EQ(map.worldToGrid(map.gridToWorld({x, y})), (GridCoord{x, y}));
        }
    }
}

void testBoundsAndOccupancyQueries() {
    GridMap map(4, 3, kResolution);
    CHECK(map.inBounds({0, 0}));
    CHECK(map.inBounds({3, 2}));
    CHECK(!map.inBounds({4, 0}));
    CHECK(!map.inBounds({0, 3}));
    CHECK(!map.inBounds({-1, 0}));
    CHECK(!map.inBounds({0, -1}));
    CHECK(map.isOccupied({-1, 0}));  // out of bounds counts as occupied
    CHECK(!map.isFree({4, 0}));
    map.setOccupied({1, 1}, true);
    CHECK(map.isOccupied({1, 1}));
    map.setOccupied({1, 1}, false);
    CHECK(map.isFree({1, 1}));
}

void testConstructorRejectsInvalidInput() {
    bool threw = false;
    try {
        GridMap map(0, 5, kResolution);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);

    threw = false;
    try {
        GridMap map(2, 2, -1.0);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);

    threw = false;
    try {
        GridMap map(2, 2, kResolution, {0, 0, 0});
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);

    threw = false;
    try {
        GridMap map(2, 1, kResolution, {0, 2});
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);
}

// TC13 - malformed CSV files produce controlled errors.
void tc13MalformedMapsProduceErrors() {
    struct Case {
        const char* description;
        const char* text;
        const char* expected_error_fragment;
    };
    const Case cases[] = {
        {"nonrectangular", "0,0,0\n0,0\n", "must be rectangular"},
        {"unsupported value", "0,2\n0,0\n", "unsupported cell value '2'"},
        {"non-numeric value", "0,a\n0,0\n", "unsupported cell value 'a'"},
        {"negative value", "0,-1\n", "unsupported cell value '-1'"},
        {"empty cell", "0,,1\n", "empty cell"},
        {"trailing comma", "0,1,\n", "empty cell"},
        {"empty stream", "", "no grid rows"},
        {"comments only", "# nothing here\n\n", "no grid rows"},
    };
    for (const Case& c : cases) {
        const auto result = parse(c.text);
        CHECK(!result.ok);
        if (result.error.find(c.expected_error_fragment) == std::string::npos) {
            ::testfw::reportFailure(__FILE__, __LINE__,
                                    std::string(c.description) + ": unexpected error '" +
                                        result.error + "'");
        }
    }

    const auto bad_resolution = parse("0,0\n", 0.0);
    CHECK(!bad_resolution.ok);
    const auto nan_resolution = parse("0,0\n", std::numeric_limits<double>::quiet_NaN());
    CHECK(!nan_resolution.ok);

    // Error messages report the line number of the offending row.
    const auto located = parse("0,0\n0,0\n0,7\n");
    CHECK(located.error.find("line 3, column 2") != std::string::npos);
}

void testFileLoaderErrors() {
    const auto missing = amr::loadGridMapCsv("/nonexistent/amr_missing_map.csv", kResolution);
    CHECK(!missing.ok);
    CHECK(missing.error.find("does not exist") != std::string::npos);

    const TempFile empty("empty.csv", "");
    const auto empty_result = amr::loadGridMapCsv(empty.path(), kResolution);
    CHECK(!empty_result.ok);
    CHECK(empty_result.error.find("is empty") != std::string::npos);

    const auto directory =
        amr::loadGridMapCsv(std::filesystem::temp_directory_path(), kResolution);
    CHECK(!directory.ok);
    CHECK(directory.error.find("not a regular file") != std::string::npos);

    const TempFile malformed("malformed.csv", "0,0\n0\n");
    const auto malformed_result = amr::loadGridMapCsv(malformed.path(), kResolution);
    CHECK(!malformed_result.ok);
    CHECK(malformed_result.error.find("amr_test_malformed.csv") != std::string::npos);
}

void testFileLoaderSuccess() {
    const TempFile valid("valid.csv", "0,1,0\n0,0,0\n1,1,0\n");
    const auto result = amr::loadGridMapCsv(valid.path(), kResolution);
    REQUIRE(result.ok);
    CHECK_EQ(result.map.width(), 3);
    CHECK_EQ(result.map.height(), 3);
    CHECK_NEAR(result.map.resolution(), kResolution, kTolerance);
    CHECK(result.map.isOccupied({1, 0}));
    CHECK(result.map.isOccupied({1, 2}));
}

/// The maps shipped in maps/ must load and connect their documented start/goal.
void testShippedSampleMaps() {
    struct Sample {
        const char* file;
        GridCoord start;
        GridCoord goal;
    };
    const Sample samples[] = {
        {"empty.csv", {1, 1}, {18, 18}},
        {"obstacles.csv", {1, 1}, {18, 18}},
        {"corridor.csv", {1, 1}, {18, 18}},
        {"maze.csv", {1, 1}, {19, 19}},
    };
    const std::filesystem::path maps_dir = std::filesystem::path(AMR_SOURCE_DIR) / "maps";
    for (const Sample& sample : samples) {
        const auto result = amr::loadGridMapCsv(maps_dir / sample.file, kResolution);
        if (!result.ok) {
            ::testfw::reportFailure(__FILE__, __LINE__, result.error);
            continue;
        }
        CHECK_EQ(result.map.width(), 20);
        CHECK_EQ(result.map.height(), 20);
        CHECK(result.map.isFree(sample.start));
        CHECK(result.map.isFree(sample.goal));
        CHECK(amr::bfsShortestPathCost(result.map, sample.start, sample.goal).has_value());
    }
}

}  // namespace

int main() {
    return testfw::runAll({
        {"Parse valid map (row-major, row 0 = top)", testParsesValidMapRowMajor},
        {"Parse tolerates whitespace, comments, CRLF", testToleratesWhitespaceCommentsAndCrlf},
        {"Grid-to-world conversion uses cell centers", testCoordinateConversion},
        {"World-to-grid conversion and round trip", testWorldToGrid},
        {"Bounds and occupancy queries", testBoundsAndOccupancyQueries},
        {"GridMap constructor rejects invalid input", testConstructorRejectsInvalidInput},
        {"TC13 Malformed CSV maps produce controlled errors", tc13MalformedMapsProduceErrors},
        {"File loader reports missing/empty/invalid files", testFileLoaderErrors},
        {"File loader loads a valid file", testFileLoaderSuccess},
        {"Shipped sample maps load and are solvable", testShippedSampleMaps},
    });
}
