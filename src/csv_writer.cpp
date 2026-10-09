#include "csv_writer.hpp"

#include <fstream>
#include <iomanip>

#include "astar.hpp"

namespace amr {

namespace {

constexpr int kMetersPrecision = 4;
constexpr int kMillisecondsPrecision = 6;

bool finishWrite(std::ofstream& out, const std::filesystem::path& file, std::string& error) {
    out.flush();
    if (!out) {
        error = "failed to write '" + file.string() + "'";
        return false;
    }
    return true;
}

}  // namespace

bool writePathCsv(const std::filesystem::path& file, const GridMap& map,
                  const std::vector<GridCoord>& path, std::string& error) {
    std::ofstream out(file);
    if (!out) {
        error = "cannot open '" + file.string() + "' for writing";
        return false;
    }
    out << "waypoint_index,grid_x,grid_y,world_x,world_y\n";
    out << std::fixed << std::setprecision(kMetersPrecision);
    for (std::size_t i = 0; i < path.size(); ++i) {
        const WorldPoint world = map.gridToWorld(path[i]);
        out << i << ',' << path[i].x << ',' << path[i].y << ',' << world.x << ','
            << world.y << '\n';
    }
    return finishWrite(out, file, error);
}

bool writePlanningMetricsCsv(const std::filesystem::path& file, const GridMap& map,
                             GridCoord start, GridCoord goal, const PlanResult& result,
                             std::string& error) {
    std::ofstream out(file);
    if (!out) {
        error = "cannot open '" + file.string() + "' for writing";
        return false;
    }
    out << "status,success,map_width,map_height,resolution,start_x,start_y,goal_x,goal_y,"
           "planning_time_ms,expanded_nodes,waypoint_count,path_length_steps,path_length_m\n";
    out << toString(result.status) << ',' << (result.success() ? 1 : 0) << ','
        << map.width() << ',' << map.height() << ','
        << std::fixed << std::setprecision(kMetersPrecision) << map.resolution() << ','
        << start.x << ',' << start.y << ',' << goal.x << ',' << goal.y << ','
        << std::setprecision(kMillisecondsPrecision) << result.planning_time_ms << ','
        << result.expanded_nodes << ',' << result.path.size() << ',';
    if (result.success()) {
        out << result.path_cost << ',' << std::setprecision(kMetersPrecision)
            << pathLengthMeters(map, result.path);
    } else {
        out << ',';
    }
    out << '\n';
    return finishWrite(out, file, error);
}

}  // namespace amr
