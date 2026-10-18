// Monte Carlo localization: a particle filter on the known map, with the existing ray
// caster as its sensor model. It can start knowing nothing (global localization) and
// recover when the robot is carried somewhere else (the kidnapped robot).
//
// Particles are poses (x, y, theta) with weights.
//
// Motion: each particle applies the measured odometry increment (turn, then drive;
// ekf.hpp) with its own noise, drawn from the odometry error model inflated by
// `motion_noise_scale` (a filter that is a little more uncertain than the truth keeps
// enough spread to follow it).
//
// Measurement: a subset of the scan's beams (`beams`, evenly spread). For each, the
// expected range is cast from the particle on the map and the reading is scored by
// the classic beam model, a mixture of
//   * a hit near the expected range:  z_hit  N(z; r_hat, sigma_hit^2)
//   * a random reading:               z_rand / max_range
//   * a reading at max_range:         z_max (when there was no return)
// The mixture keeps one surprising beam (an unmapped obstacle, a reflection) from
// killing an otherwise good particle. sigma_hit adapts per place (annealing): while a
// cloud of particles is coarse, its nearest particle is far from the truth, and a
// sharp likelihood would decide between places by the luck of where particles
// happen to be. The particles are scored with sigma = clamp(spread of the places
// that carry weight (clusters, see below; heading counted as range error at 3 m),
// sigma_hit, sigma_hit_max): wide during global localization, sharp once converged,
// and the same for every place (likelihoods of different widths are not comparable). Beams of one scan are not independent (section
// 17 of docs/algorithms.md), so the scan's log-likelihood is tempered: divided by
// `correlation` (beams per independent piece of information).
//
// Resampling happens when the effective sample size drops below half the particles,
// and is low-variance (systematic: n evenly spaced pointers into the cumulative
// weights): independent draws add sampling noise that, on a symmetric map, lets one
// of two equally good places die out at random (mode collapse). The number of
// particles adapts (KLD-sampling, Fox 2003): enough to keep the sampling error of
// the particle distribution, measured on a histogram of 0.25 m x 0.25 m x 10 deg
// bins (those holding weight), below kld_error with probability kld_quantile.
// Global localization starts with global_particles (50 000) spread uniformly over the
// free cells, about one per 15 cm x 15 cm x 5 deg here: with fewer, often no particle
// starts near the truth and the filter commits to the best-fitting wrong place. The
// first resampling brings them down to at most max_particles; once they agree on one
// place, a few hundred suffice.
//
// Kidnapping (augmented MCL, Thrun et al. 2005). Short- and long-term averages of the
// per-beam measurement likelihood of the best-fitting place (at the sharp sigma_hit)
// are kept (alpha_fast, alpha_slow). When the short-term average drops well below
// the long-term one, the measurements stopped fitting the particles: a fraction
// max(0, 1 - fast / (inject_deadband x slow)) of the resampled particles is replaced
// by poses drawn uniformly over the free space. They enter with negligible weight
// and earn weight only by fitting the next scan. The deadband (0.9) keeps a fresh
// cloud, which fits a little worse than a long-converged one, from injecting on
// every scan. When that fraction exceeds
// `relocalize` (the scans fit less than half as well as they used to), the filter
// starts global localization over (global_particles); a few thousand random
// particles are too sparse to land near the new pose quickly.
//
// Several matching places (keep_modes). On a symmetric map two places see exactly
// the same data, but early on their particle clouds are coarse: whichever happens to
// have particles nearer its exact pose wins the first sharp updates, and resampling
// then starves the other (it died within 4 scans in half the runs). So the
// particles are clustered into places (connected 0.5 m x 0.5 m x 30 deg bins) and
// each place is resampled on its own, with at least min_mode_particles and keeping
// its total weight (a mixture particle filter, Vermaak et al. 2003). Places below
// mode_floor are dropped, and at most max_modes are kept. Even so, coarse clouds make
// the place weights drift by luck in the first scans (a place's few particles with
// nearly the right heading decide its score). So while the places are coarser than
// refine_spread, each scan first refines every place on its own: three annealing
// layers (sigma 0.6, 0.3, 0.15 m) re-weight and resample particles within their
// place, with diffusion, without changing any place's weight. Places are compared
// only once each cloud sits on its own best fit.
//
// Estimate: the particles are binned on a 0.5 m grid; the best 3 x 3 block of bins
// (most weight) is the dominant mode. Its weighted mean (circular mean for heading)
// is the estimate, its weight share says how sure the filter is of that place.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

#include "ekf.hpp"
#include "grid_map.hpp"
#include "lidar.hpp"

namespace amr {

struct ParticleFilterConfig {
    std::size_t global_particles = 50000;  ///< global localization: particles for the first scan
    std::size_t max_particles = 5000;
    std::size_t min_particles = 500;
    std::size_t beams = 36;           ///< beams used per update (subset of the scan)
    double sigma_hit = 0.05;          ///< m, once the particles agree on one place
    double sigma_hit_max = 0.3;       ///< m, while they are spread or split
    double z_hit = 0.85;
    double z_rand = 0.1;
    double z_max = 0.05;
    double correlation = 10.0;        ///< beams per independent piece of information
    double motion_noise_scale = 1.5;  ///< odometry noise std multiplier for the particles
    double kld_error = 0.01;          ///< KLD-sampling bound
    double kld_quantile = 2.326;      ///< upper standard-normal quantile (99 %)
    bool recover = true;              ///< inject random particles when the scans stop fitting
    double alpha_slow = 0.001;
    double alpha_fast = 0.1;
    double relocalize = 0.5;          ///< injection fraction that triggers a global restart
    double inject_deadband = 0.9;     ///< inject only when the fit drops below this x the long-term fit
    bool keep_modes = true;           ///< resample each place (cluster) on its own, keeping its weight
    double mode_floor = 1e-4;         ///< places with less weight are dropped
    std::size_t max_modes = 20;
    std::size_t min_mode_particles = 100;
    double refine_spread = 0.15;      ///< refine places whose clouds are coarser than this, m
    std::uint32_t seed = 0;
};

struct Particle {
    Pose pose;
    double weight = 1.0;
};

/// The dominant mode of the particles.
struct ParticleEstimate {
    Pose pose;
    double share = 0.0;        ///< weight of the mode (0..1)
    double position_std = 0.0; ///< spread of the mode, m
    double heading_std = 0.0;  ///< rad
};

class ParticleFilter {
public:
    ParticleFilter(const GridMap& map, const ParticleFilterConfig& config);

    /// Uniform over the free cells, any heading (global localization).
    void initializeGlobal();
    /// Around a pose (known start), with the given standard deviations.
    void initializeAt(const Pose& pose, double position_sigma, double heading_sigma);

    /// Motion update with a measured increment and its variances (as PoseEkf::predict).
    void predict(const OdometryIncrement& measured, const std::array<double, 2>& variances);

    /// Measurement update; `offsets` are beam angles relative to the heading.
    void update(const std::vector<double>& offsets, const std::vector<double>& ranges,
                const std::vector<bool>& hits, double max_range);

    ParticleEstimate estimate() const;
    const std::vector<Particle>& particles() const { return particles_; }
    std::size_t injected() const { return injected_; }  ///< random particles added so far
    std::size_t relocalizations() const { return relocalizations_; }
    std::size_t refinements() const { return refinements_; }  ///< scans with place refinement

private:
    Pose randomFreePose();
    void resample(double inject_fraction);
    void resampleModes(double inject_fraction);
    /// Annealed refinement of each place's cloud on one scan (place weights unchanged).
    void refinePlaces(const std::vector<std::size_t>& used, const std::vector<double>& offsets,
                      const std::vector<double>& ranges, const std::vector<bool>& hits, double max_range);

    const GridMap& map_;
    ParticleFilterConfig config_;
    std::mt19937 rng_;
    std::vector<Particle> particles_;
    std::vector<GridCoord> free_cells_;
    double w_slow_ = 0.0;
    double w_fast_ = 0.0;
    std::size_t injected_ = 0;
    std::size_t relocalizations_ = 0;
    std::size_t refinements_ = 0;
};

/// One sample of a particle-filter localization run.
struct ParticleSample {
    double time = 0.0;
    Pose truth;
    ParticleEstimate estimate;
    std::size_t particles = 0;
    std::size_t relocalizations = 0;  ///< global restarts so far
    std::size_t refinements = 0;      ///< scans with place refinement so far
};

/// Run the particle filter along a trajectory with the same simulated odometry as
/// runLocalization (seeded by ekf.odometry.seed). `kidnapped[k]` true: between sample
/// k - 1 and k the robot was carried away (the odometry reports no motion).
std::vector<ParticleSample> runParticleFilter(const GridMap& map,
                                              const std::vector<TrajectorySample>& trajectory,
                                              const std::vector<LidarScan>& scans,
                                              const LidarConfig& lidar, const OdometryNoise& odometry,
                                              const ParticleFilterConfig& config, bool global_start,
                                              const std::vector<bool>& kidnapped = {});

}  // namespace amr
