#include "csv_writer.hpp"

#include <fstream>
#include <iomanip>

#include "astar.hpp"

namespace amr {

namespace {

constexpr int kMetersPrecision = 4;
constexpr int kMillisecondsPrecision = 6;
constexpr int kSecondsPrecision = 6;
constexpr int kCellsPrecision = 6;
constexpr int kAnglePrecision = 9;
constexpr int kCovariancePrecision = 6;  // significant digits after the first
constexpr int kNeesPrecision = 4;

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
                             const ReferenceCost& reference, double robot_radius,
                             double inflation_time_ms, Connectivity connectivity,
                             std::string& error) {
    std::ofstream out(file);
    if (!out) {
        error = "cannot open '" + file.string() + "' for writing";
        return false;
    }
    out << "status,success,map_width,map_height,resolution,start_x,start_y,goal_x,goal_y,"
           "planning_time_ms,expanded_nodes,waypoint_count,path_length_steps,path_length_m,"
           "reference_bfs_cost,robot_radius_m,inflation_time_ms,connectivity,diagonal_steps,"
           "path_cost_cells\n";
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
    out << ',';
    if (reference.computed && !reference.cost) {
        out << kReferenceNoPath;
    } else if (reference.computed && connectivity == Connectivity::Four) {
        out << reference.cost->straight;
    } else if (reference.computed) {
        out << std::setprecision(kCellsPrecision) << reference.cost->cells();
    }
    out << ',' << std::setprecision(kMetersPrecision) << robot_radius << ','
        << std::setprecision(kMillisecondsPrecision) << inflation_time_ms << ','
        << static_cast<int>(connectivity) << ',';
    if (result.success()) {
        out << result.diagonal_moves << ',' << std::setprecision(kCellsPrecision)
            << result.cost().cells();
    } else {
        out << ',';
    }
    out << '\n';
    return finishWrite(out, file, error);
}

bool writeLidarCsv(const std::filesystem::path& file, const std::vector<LidarScan>& scans,
                   std::string& error) {
    std::ofstream out(file);
    if (!out) {
        error = "cannot open '" + file.string() + "' for writing";
        return false;
    }
    out << "time,beam,angle,range,hit\n" << std::fixed;
    for (const LidarScan& scan : scans) {
        for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
            out << std::setprecision(kSecondsPrecision) << scan.time << ',' << i << ','
                << std::setprecision(kAnglePrecision) << scan.angles[i] << ','
                << std::setprecision(kMillisecondsPrecision) << scan.ranges[i] << ','
                << (scan.hits[i] ? 1 : 0) << '\n';
        }
    }
    return finishWrite(out, file, error);
}

bool writeLocalizationCsv(const std::filesystem::path& file,
                          const std::vector<LocalizationSample>& samples, std::string& error) {
    std::ofstream out(file);
    if (!out) {
        error = "cannot open '" + file.string() + "' for writing";
        return false;
    }
    out << "time,true_x,true_y,true_theta,odom_x,odom_y,odom_theta,est_x,est_y,est_theta,"
           "var_x,cov_xy,var_y,cov_x_theta,cov_y_theta,var_theta,beams_used,beams_rejected,"
           "nees\n";
    const auto pose = [&out](const Pose& p) {
        out << std::fixed << std::setprecision(kMillisecondsPrecision) << p.x << ',' << p.y
            << ',' << std::setprecision(kAnglePrecision) << p.theta << ',';
    };
    for (const LocalizationSample& s : samples) {
        out << std::fixed << std::setprecision(kSecondsPrecision) << s.time << ',';
        pose(s.truth);
        pose(s.odometry);
        pose(s.estimate);
        // Upper triangle of the covariance: xx, xy, yy, x theta, y theta, theta theta.
        out << std::scientific << std::setprecision(kCovariancePrecision) << s.covariance[0]
            << ',' << s.covariance[1] << ',' << s.covariance[4] << ',' << s.covariance[2] << ','
            << s.covariance[5] << ',' << s.covariance[8] << ',' << s.update.used << ','
            << s.update.rejected << ',' << std::fixed << std::setprecision(kNeesPrecision)
            << s.nees << '\n';
    }
    return finishWrite(out, file, error);
}

bool writeDynamicObstaclesCsv(const std::filesystem::path& file, const GridMap& map,
                              const std::vector<TrajectorySample>& trajectory,
                              const std::vector<DynamicObstacle>& obstacles, double sensing_range,
                              std::string& error) {
    std::ofstream out(file);
    if (!out) {
        error = "cannot open '" + file.string() + "' for writing";
        return false;
    }
    out << "time,obstacle,x,y,radius,visible\n" << std::fixed;
    for (const TrajectorySample& sample : trajectory) {
        const WorldPoint robot{sample.state.x, sample.state.y};
        for (std::size_t i = 0; i < obstacles.size(); ++i) {
            const WorldPoint p = obstaclePosition(obstacles[i], sample.state.time);
            const bool visible =
                isObstacleVisible(map, robot, p, obstacles[i].radius, sensing_range);
            out << std::setprecision(kSecondsPrecision) << sample.state.time << ',' << i << ','
                << std::setprecision(kMillisecondsPrecision) << p.x << ',' << p.y << ','
                << std::setprecision(kMetersPrecision) << obstacles[i].radius << ','
                << (visible ? 1 : 0) << '\n';
        }
    }
    return finishWrite(out, file, error);
}

bool writeReplansCsv(const std::filesystem::path& events_file,
                     const std::filesystem::path& paths_file,
                     const std::vector<ReplanEvent>& replans, std::string& error) {
    std::ofstream events(events_file);
    std::ofstream paths(paths_file);
    if (!events || !paths) {
        error = "cannot open '" + (events ? paths_file : events_file).string() + "' for writing";
        return false;
    }
    events << "replan,time,from_x,from_y,status,waypoints,path_cost_cells,expanded_nodes,"
              "planning_time_ms,blocked_cells\n"
           << std::fixed;
    paths << "replan,waypoint_index,grid_x,grid_y\n";
    for (std::size_t i = 0; i < replans.size(); ++i) {
        const ReplanEvent& e = replans[i];
        events << i << ',' << std::setprecision(kSecondsPrecision) << e.time << ',' << e.from.x
               << ',' << e.from.y << ',' << toString(e.status) << ',';
        if (e.status == PlanStatus::Success) {
            events << e.path.size() << ',' << std::setprecision(kCellsPrecision) << e.cost.cells();
        } else {
            events << ',';
        }
        events << ',' << e.expanded_nodes << ',' << std::setprecision(kMillisecondsPrecision)
               << e.planning_time_ms << ',' << e.blocked_cells << '\n';
        for (std::size_t k = 0; k < e.path.size(); ++k) {
            paths << i << ',' << k << ',' << e.path[k].x << ',' << e.path[k].y << '\n';
        }
    }
    return finishWrite(events, events_file, error) && finishWrite(paths, paths_file, error);
}

bool writeGridMapCsv(const std::filesystem::path& file, const GridMap& map, std::string& error) {
    std::ofstream out(file);
    if (!out) {
        error = "cannot open '" + file.string() + "' for writing";
        return false;
    }
    for (int y = 0; y < map.height(); ++y) {
        for (int x = 0; x < map.width(); ++x) {
            out << (x > 0 ? "," : "") << (map.isOccupied({x, y}) ? '1' : '0');
        }
        out << '\n';
    }
    return finishWrite(out, file, error);
}

bool writeTrajectoryCsv(const std::filesystem::path& file,
                        const std::vector<TrajectorySample>& trajectory, bool with_heading,
                        std::string& error) {
    std::ofstream out(file);
    if (!out) {
        error = "cannot open '" + file.string() + "' for writing";
        return false;
    }
    out << "time,x,y,target_waypoint_index"
        << (with_heading ? ",theta,linear_velocity,angular_velocity" : "") << '\n';
    out << std::fixed;
    for (const TrajectorySample& sample : trajectory) {
        out << std::setprecision(kSecondsPrecision) << sample.state.time << ','
            << std::setprecision(kMillisecondsPrecision) << sample.state.x << ','
            << sample.state.y << ',' << sample.target_waypoint_index;
        if (with_heading) {
            out << ',' << std::setprecision(kAnglePrecision) << sample.state.theta << ','
                << sample.linear_velocity << ',' << sample.angular_velocity;
        }
        out << '\n';
    }
    return finishWrite(out, file, error);
}

bool writeSimulationMetricsCsv(const std::filesystem::path& file, bool planning_succeeded,
                               const MotionConfig& config, const SimulationResult& result,
                               std::string& error) {
    std::ofstream out(file);
    if (!out) {
        error = "cannot open '" + file.string() + "' for writing";
        return false;
    }
    out << "status,success,navigation_success,steps,travel_time_s,distance_traveled_m,"
           "final_goal_error_m,collision_count,speed_mps,dt_s,goal_tolerance_m,max_time_s,"
           "robot_radius_m,robot_model,turns,turn_time_s,total_rotation_rad,final_heading_rad\n";
    const bool ran = !result.trajectory.empty();
    out << toString(result.status) << ',' << (result.success() ? 1 : 0) << ','
        << (planning_succeeded && result.success() ? 1 : 0) << ',' << result.steps << ',';
    out << std::fixed;
    if (ran) {
        out << std::setprecision(kSecondsPrecision) << result.travelTime() << ','
            << std::setprecision(kMillisecondsPrecision) << result.distance_traveled << ','
            << result.final_goal_error << ',';
    } else {
        out << ",,,";
    }
    out << result.collision_count << ',' << std::setprecision(kMillisecondsPrecision)
        << config.speed << ',' << config.dt << ',' << config.goal_tolerance << ','
        << config.max_time << ',' << config.robot_radius << ',';
    if (config.model == RobotModel::DiffDrive) {
        out << "diff-drive," << result.turns << ',' << result.turn_time << ','
            << result.total_rotation << ',' << std::setprecision(kAnglePrecision)
            << (ran ? result.trajectory.back().state.theta : config.initial_heading) << '\n';
    } else {
        out << "point,,,,\n";
    }
    return finishWrite(out, file, error);
}

}  // namespace amr
