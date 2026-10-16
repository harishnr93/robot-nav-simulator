#include "particle_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

#include "diff_drive.hpp"

namespace amr {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kSqrt2Pi = 2.5066282746310002;

long long binKey(int x, int y, int z) {
    return ((static_cast<long long>(x) + 100000) * 200003LL + (static_cast<long long>(y) + 100000)) * 64 + z;
}

/// KLD-sampling: particles needed for k occupied bins (Fox 2003).
double kldBound(std::size_t k, double error, double quantile) {
    if (k <= 1) {
        return 0.0;
    }
    const double km1 = static_cast<double>(k - 1);
    const double a = 2.0 / (9.0 * km1);
    const double b = 1.0 - a + std::sqrt(a) * quantile;
    return km1 / (2.0 * error) * b * b * b;
}

}  // namespace

namespace {

/// Clusters of particles: connected cells of a 0.5 m x 0.5 m x 30 deg grid
/// (26-neighbourhood, heading wrapping). Returns a cluster index per particle.
std::vector<int> clusterParticles(const std::vector<Particle>& particles, int& count) {
    std::unordered_map<long long, int> cell_of;
    std::vector<std::array<int, 3>> cells;
    std::vector<int> particle_cell(particles.size());
    for (std::size_t i = 0; i < particles.size(); ++i) {
        const Pose& p = particles[i].pose;
        const std::array<int, 3> c{static_cast<int>(std::floor(p.x / 0.5)), static_cast<int>(std::floor(p.y / 0.5)),
                                   static_cast<int>(std::floor((p.theta + kPi) / (kPi / 6.0))) % 12};
        const long long key = binKey(c[0], c[1], c[2]);
        auto it = cell_of.find(key);
        if (it == cell_of.end()) {
            it = cell_of.emplace(key, static_cast<int>(cells.size())).first;
            cells.push_back(c);
        }
        particle_cell[i] = it->second;
    }
    std::vector<int> label(cells.size(), -1);
    count = 0;
    for (std::size_t start = 0; start < cells.size(); ++start) {
        if (label[start] >= 0) {
            continue;
        }
        std::vector<int> stack{static_cast<int>(start)};
        label[start] = count;
        while (!stack.empty()) {
            const std::array<int, 3> c = cells[static_cast<std::size_t>(stack.back())];
            stack.pop_back();
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        const auto it = cell_of.find(binKey(c[0] + dx, c[1] + dy, (c[2] + dz + 12) % 12));
                        if (it != cell_of.end() && label[static_cast<std::size_t>(it->second)] < 0) {
                            label[static_cast<std::size_t>(it->second)] = count;
                            stack.push_back(it->second);
                        }
                    }
                }
            }
        }
        ++count;
    }
    std::vector<int> result(particles.size());
    for (std::size_t i = 0; i < particles.size(); ++i) {
        result[i] = label[static_cast<std::size_t>(particle_cell[i])];
    }
    return result;
}

}  // namespace

ParticleFilter::ParticleFilter(const GridMap& map, const ParticleFilterConfig& config)
    : map_(map), config_(config), rng_(config.seed) {
    for (int y = 0; y < map.height(); ++y) {
        for (int x = 0; x < map.width(); ++x) {
            if (map.isFree({x, y})) {
                free_cells_.push_back({x, y});
            }
        }
    }
}

Pose ParticleFilter::randomFreePose() {
    std::uniform_int_distribution<std::size_t> cell(0, free_cells_.size() - 1);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    const GridCoord c = free_cells_[cell(rng_)];
    const double r = map_.resolution();
    return {(c.x + unit(rng_)) * r, (c.y + unit(rng_)) * r, wrapAngle(2.0 * kPi * unit(rng_) - kPi)};
}

void ParticleFilter::initializeGlobal() {
    particles_.clear();
    const std::size_t n = std::max(config_.global_particles, config_.max_particles);
    const double w = 1.0 / static_cast<double>(n);
    for (std::size_t i = 0; i < n; ++i) {
        particles_.push_back({randomFreePose(), w});
    }
    w_slow_ = w_fast_ = 0.0;
}

void ParticleFilter::initializeAt(const Pose& pose, double position_sigma, double heading_sigma) {
    particles_.clear();
    std::normal_distribution<double> unit(0.0, 1.0);
    const std::size_t n = config_.min_particles * 2;
    for (std::size_t i = 0; i < n; ++i) {
        particles_.push_back({{pose.x + position_sigma * unit(rng_), pose.y + position_sigma * unit(rng_),
                               wrapAngle(pose.theta + heading_sigma * unit(rng_))},
                              1.0 / static_cast<double>(n)});
    }
    w_slow_ = w_fast_ = 0.0;
}

void ParticleFilter::predict(const OdometryIncrement& measured, const std::array<double, 2>& variances) {
    std::normal_distribution<double> unit(0.0, 1.0);
    const double s = config_.motion_noise_scale;
    const double sd = s * std::sqrt(variances[0]);
    const double sr = s * std::sqrt(variances[1]);
    for (Particle& p : particles_) {
        const OdometryIncrement noisy{measured.distance + sd * unit(rng_), measured.rotation + sr * unit(rng_)};
        p.pose = applyIncrement(p.pose, noisy);
    }
}

void ParticleFilter::update(const std::vector<double>& offsets, const std::vector<double>& ranges,
                            const std::vector<bool>& hits, double max_range) {
    if (particles_.empty() || offsets.empty()) {
        return;
    }
    // Evenly spread subset of the beams.
    std::vector<std::size_t> used;
    const std::size_t step = std::max<std::size_t>(1, offsets.size() / std::max<std::size_t>(1, config_.beams));
    for (std::size_t i = 0; i < offsets.size(); i += step) {
        used.push_back(i);
    }
    if (config_.keep_modes) {
        refinePlaces(used, offsets, ranges, hits, max_range);
    }
    // Annealing per place: each cluster of particles is scored with a likelihood as
    // wide as its own spread (position, and heading as range error at 3 m), so a
    // coarse cloud neither wins nor loses big by the luck of where its particles are.
    int cluster_count = 0;
    const std::vector<int> cluster = clusterParticles(particles_, cluster_count);
    std::vector<double> cw(static_cast<std::size_t>(cluster_count), 0.0), cx(cw), cy(cw), cs(cw), cc(cw), cv(cw), ct(cw);
    for (std::size_t i = 0; i < particles_.size(); ++i) {
        const auto c = static_cast<std::size_t>(cluster[i]);
        const Particle& p = particles_[i];
        cw[c] += p.weight;
        cx[c] += p.weight * p.pose.x;
        cy[c] += p.weight * p.pose.y;
        cs[c] += p.weight * std::sin(p.pose.theta);
        cc[c] += p.weight * std::cos(p.pose.theta);
    }
    for (std::size_t i = 0; i < particles_.size(); ++i) {
        const auto c = static_cast<std::size_t>(cluster[i]);
        const Particle& p = particles_[i];
        const double dx = p.pose.x - cx[c] / cw[c];
        const double dy = p.pose.y - cy[c] / cw[c];
        const double dt = wrapAngle(p.pose.theta - std::atan2(cs[c], cc[c]));
        cv[c] += p.weight * (dx * dx + dy * dy);
        ct[c] += p.weight * dt * dt;
    }
    // One width for all places (likelihoods of different widths are not comparable):
    // the weighted spread of the places that carry weight.
    double spread_sum = 0.0, spread_weight = 0.0, total_weight = 0.0;
    for (std::size_t c = 0; c < cw.size(); ++c) {
        total_weight += cw[c];
    }
    for (std::size_t c = 0; c < cw.size(); ++c) {
        if (cw[c] > 0.01 * total_weight) {
            spread_sum += cw[c] * std::sqrt(cv[c] / cw[c] + 9.0 * ct[c] / cw[c]);
            spread_weight += cw[c];
        }
    }
    const double common = std::clamp(spread_weight > 0.0 ? spread_sum / spread_weight : config_.sigma_hit_max,
                                     config_.sigma_hit, config_.sigma_hit_max);
    std::vector<double> sigma_of(static_cast<std::size_t>(cluster_count), common);
    const double rand_density = config_.z_rand / max_range;
    std::vector<double> log_likelihood(particles_.size());
    double best = -std::numeric_limits<double>::infinity();
    // Kidnap detection uses the per-beam likelihood at the fixed, sharp sigma_hit (with
    // the annealed width it would also move whenever the width does).
    double average = 0.0;
    std::vector<double> place_fit(static_cast<std::size_t>(cluster_count), 0.0);
    const double fixed_var = config_.sigma_hit * config_.sigma_hit;
    for (std::size_t k = 0; k < particles_.size(); ++k) {
        const Pose& pose = particles_[k].pose;
        const double sigma = sigma_of[static_cast<std::size_t>(cluster[k])];
        const double var = sigma * sigma;
        double sum = 0.0, fixed = 0.0;
        if (!map_.isFree(map_.worldToGrid({pose.x, pose.y}))) {
            sum = fixed = -1e3 * static_cast<double>(used.size());  // inside an obstacle: impossible
        } else {
            for (const std::size_t i : used) {
                const RayHit expected = castRay(map_, {pose.x, pose.y}, pose.theta + offsets[i], max_range);
                const double z = hits[i] ? ranges[i] : max_range;
                const double r_hat = expected.hit ? expected.range : max_range;
                const double d = z - r_hat;
                const double extra = rand_density + (hits[i] ? 0.0 : config_.z_max);
                sum += std::log(extra + config_.z_hit * std::exp(-0.5 * d * d / var) / (kSqrt2Pi * sigma));
                fixed += std::log(extra + config_.z_hit * std::exp(-0.5 * d * d / fixed_var) /
                                              (kSqrt2Pi * config_.sigma_hit));
            }
        }
        place_fit[static_cast<std::size_t>(cluster[k])] += particles_[k].weight * std::exp(fixed / static_cast<double>(used.size()));
        log_likelihood[k] = sum / config_.correlation;
        best = std::max(best, log_likelihood[k]);
    }
    // The best place's fit (among places with weight): injected random particles,
    // which carry little information, must not hold the statistic down (they would
    // keep the injection going, which keeps them coming).
    for (std::size_t c = 0; c < place_fit.size(); ++c) {
        if (cw[c] > 0.01 * total_weight) {
            average = std::max(average, place_fit[c] / cw[c]);
        }
    }
    double total = 0.0;
    for (std::size_t k = 0; k < particles_.size(); ++k) {
        particles_[k].weight *= std::exp(log_likelihood[k] - best);
        total += particles_[k].weight;
    }
    double sum_sq = 0.0;
    for (Particle& p : particles_) {
        p.weight /= total;
        sum_sq += p.weight * p.weight;
    }
    // Augmented MCL: short- and long-term averages of the measurement likelihood.
    if (w_slow_ == 0.0) {
        w_slow_ = w_fast_ = average;
    } else {
        w_slow_ += config_.alpha_slow * (average - w_slow_);
        w_fast_ += config_.alpha_fast * (average - w_fast_);
    }
    // Deadband: a fit within inject_deadband of the long-term level is no evidence of a
    // kidnapping (a fresh cloud fits a little worse than a long-converged one), and
    // injecting on every scan would only waste particles.
    const double inject =
        config_.recover ? std::max(0.0, 1.0 - w_fast_ / (config_.inject_deadband * w_slow_)) : 0.0;
    if (inject > config_.relocalize) {
        initializeGlobal();  // kidnapped: start over (the next scan weighs the new particles)
        ++relocalizations_;
        return;
    }
    const double ess = 1.0 / sum_sq;
    if (config_.keep_modes || ess < 0.5 * static_cast<double>(particles_.size()) || inject > 0.0) {
        resample(inject);
    }
}


void ParticleFilter::refinePlaces(const std::vector<std::size_t>& used, const std::vector<double>& offsets,
                                  const std::vector<double>& ranges, const std::vector<bool>& hits,
                                  double max_range) {
    std::normal_distribution<double> jitter(0.0, 1.0);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    const double rand_density = config_.z_rand / max_range;
    bool counted = false;
    for (const double sigma : {0.6, 0.3, 0.15}) {
        int count = 0;
        const std::vector<int> cluster = clusterParticles(particles_, count);
        std::vector<double> w(static_cast<std::size_t>(count), 0.0), mx(w), my(w), v(w);
        std::vector<std::size_t> size(static_cast<std::size_t>(count), 0);
        double total = 0.0;
        for (std::size_t i = 0; i < particles_.size(); ++i) {
            const auto c = static_cast<std::size_t>(cluster[i]);
            w[c] += particles_[i].weight;
            mx[c] += particles_[i].weight * particles_[i].pose.x;
            my[c] += particles_[i].weight * particles_[i].pose.y;
            ++size[c];
            total += particles_[i].weight;
        }
        for (std::size_t i = 0; i < particles_.size(); ++i) {
            const auto c = static_cast<std::size_t>(cluster[i]);
            const double dx = particles_[i].pose.x - mx[c] / w[c];
            const double dy = particles_[i].pose.y - my[c] / w[c];
            v[c] += particles_[i].weight * (dx * dx + dy * dy);
        }
        // Coarse places: real clouds (several particles, some weight) spread wider than
        // refine_spread. Converged places are left alone.
        std::vector<bool> coarse(static_cast<std::size_t>(count), false);
        bool any = false;
        for (std::size_t c = 0; c < coarse.size(); ++c) {
            coarse[c] = size[c] >= 10 && w[c] > 0.01 * total && std::sqrt(v[c] / w[c]) > config_.refine_spread;
            any = any || coarse[c];
        }
        if (!any) {
            return;
        }
        if (!counted) {
            ++refinements_;
            counted = true;
        }
        const double var = sigma * sigma;
        std::vector<std::vector<std::size_t>> members(static_cast<std::size_t>(count));
        for (std::size_t i = 0; i < particles_.size(); ++i) {
            if (coarse[static_cast<std::size_t>(cluster[i])]) {
                members[static_cast<std::size_t>(cluster[i])].push_back(i);
            }
        }
        for (std::size_t c = 0; c < members.size(); ++c) {
            if (members[c].empty()) {
                continue;
            }
            // Re-weight the place's particles by the scan at this layer's width.
            std::vector<double> ll(members[c].size());
            double best = -std::numeric_limits<double>::infinity();
            for (std::size_t j = 0; j < members[c].size(); ++j) {
                const Pose& pose = particles_[members[c][j]].pose;
                double sum = 0.0;
                if (!map_.isFree(map_.worldToGrid({pose.x, pose.y}))) {
                    sum = -1e3 * static_cast<double>(used.size());
                } else {
                    for (const std::size_t i : used) {
                        const RayHit expected = castRay(map_, {pose.x, pose.y}, pose.theta + offsets[i], max_range);
                        const double z = hits[i] ? ranges[i] : max_range;
                        const double d = z - (expected.hit ? expected.range : max_range);
                        double p = rand_density + config_.z_hit * std::exp(-0.5 * d * d / var) / (kSqrt2Pi * sigma);
                        if (!hits[i]) {
                            p += config_.z_max;
                        }
                        sum += std::log(p);
                    }
                }
                ll[j] = sum / config_.correlation;
                best = std::max(best, ll[j]);
            }
            std::vector<double> cumulative(members[c].size());
            double within = 0.0;
            for (std::size_t j = 0; j < members[c].size(); ++j) {
                within += particles_[members[c][j]].weight * std::exp(ll[j] - best);
                cumulative[j] = within;
            }
            // Resample within the place (systematic), keep its weight, diffuse at the
            // layer's scale so the cloud can move onto the fit.
            const std::size_t m = members[c].size();
            std::vector<Pose> poses;
            poses.reserve(m);
            const double step = within / static_cast<double>(m);
            double pointer = unit(rng_) * step;
            std::size_t k = 0;
            for (std::size_t j = 0; j < m; ++j) {
                while (cumulative[k] < pointer && k + 1 < m) {
                    ++k;
                }
                pointer += step;
                Pose pose = particles_[members[c][k]].pose;
                pose.x += 0.3 * sigma * jitter(rng_);
                pose.y += 0.3 * sigma * jitter(rng_);
                pose.theta = wrapAngle(pose.theta + 0.1 * sigma * jitter(rng_));
                poses.push_back(pose);
            }
            for (std::size_t j = 0; j < m; ++j) {
                particles_[members[c][j]] = {poses[j], w[c] / static_cast<double>(m)};
            }
        }
    }
}

void ParticleFilter::resampleModes(double inject_fraction) {
    int count = 0;
    const std::vector<int> cluster = clusterParticles(particles_, count);
    std::vector<double> weight(static_cast<std::size_t>(count), 0.0);
    double total = 0.0;
    for (std::size_t i = 0; i < particles_.size(); ++i) {
        weight[static_cast<std::size_t>(cluster[i])] += particles_[i].weight;
        total += particles_[i].weight;
    }
    // The strongest places above the floor.
    std::vector<int> order(static_cast<std::size_t>(count));
    for (int c = 0; c < count; ++c) {
        order[static_cast<std::size_t>(c)] = c;
    }
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return weight[static_cast<std::size_t>(a)] > weight[static_cast<std::size_t>(b)];
    });
    std::vector<int> kept;
    double kept_weight = 0.0;
    for (const int c : order) {
        if (kept.size() >= config_.max_modes || weight[static_cast<std::size_t>(c)] < config_.mode_floor * total) {
            break;
        }
        kept.push_back(c);
        kept_weight += weight[static_cast<std::size_t>(c)];
    }
    // Resample the kept particles as one set (for its size), then within each place.
    std::vector<Particle> pool;
    std::vector<int> pool_cluster;
    for (std::size_t i = 0; i < particles_.size(); ++i) {
        if (std::find(kept.begin(), kept.end(), cluster[i]) != kept.end()) {
            pool.push_back(particles_[i]);
            pool_cluster.push_back(cluster[i]);
        }
    }
    std::unordered_map<long long, bool> bins;
    for (const Particle& p : pool) {
        bins[binKey(static_cast<int>(std::floor(p.pose.x / 0.25)), static_cast<int>(std::floor(p.pose.y / 0.25)),
                    static_cast<int>(std::floor((p.pose.theta + kPi) / (10.0 * kPi / 180.0))))] = true;
    }
    const auto wanted = static_cast<std::size_t>(std::ceil(kldBound(bins.size(), config_.kld_error, config_.kld_quantile)));
    const std::size_t n = std::clamp(wanted, config_.min_particles, config_.max_particles);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::normal_distribution<double> jitter(0.0, 1.0);
    std::vector<Particle> next;
    for (const int c : kept) {
        const double share = weight[static_cast<std::size_t>(c)] / kept_weight;
        const std::size_t m = std::max(config_.min_mode_particles,
                                       static_cast<std::size_t>(std::lround(share * static_cast<double>(n))));
        std::vector<const Particle*> members;
        double member_total = 0.0;
        for (std::size_t i = 0; i < pool.size(); ++i) {
            if (pool_cluster[i] == c) {
                members.push_back(&pool[i]);
                member_total += pool[i].weight;
            }
        }
        const double step = member_total / static_cast<double>(m);
        double pointer = unit(rng_) * step;
        double cumulative = members.front()->weight;
        std::size_t k = 0;
        for (std::size_t j = 0; j < m; ++j) {
            while (cumulative < pointer && k + 1 < members.size()) {
                cumulative += members[++k]->weight;
            }
            pointer += step;
            Pose pose = members[k]->pose;
            double particle_weight = share / static_cast<double>(m);  // the place keeps its weight
            if (unit(rng_) < inject_fraction) {
                pose = randomFreePose();
                particle_weight *= 1e-3;  // a guess: it earns weight only by fitting the next scan
                ++injected_;
            } else {
                pose.x += 0.005 * jitter(rng_);
                pose.y += 0.005 * jitter(rng_);
                pose.theta = wrapAngle(pose.theta + 0.002 * jitter(rng_));
            }
            next.push_back({pose, particle_weight});
        }
    }
    particles_ = std::move(next);
}

void ParticleFilter::resample(double inject_fraction) {
    if (config_.keep_modes) {
        resampleModes(inject_fraction);
        return;
    }
    // Size (KLD-sampling) from the histogram the resampled set will have: the bins
    // holding a non-negligible share of the weight now.
    std::unordered_map<long long, double> bins;
    for (const Particle& p : particles_) {
        const int bx = static_cast<int>(std::floor(p.pose.x / 0.25));
        const int by = static_cast<int>(std::floor(p.pose.y / 0.25));
        const int bz = static_cast<int>(std::floor((p.pose.theta + kPi) / (10.0 * kPi / 180.0)));
        bins[binKey(bx, by, bz)] += p.weight;
    }
    std::size_t occupied = 0;
    double total = 0.0;
    for (const Particle& p : particles_) {
        total += p.weight;
    }
    for (const auto& [key, weight] : bins) {
        (void)key;
        occupied += weight > 0.1 * total / static_cast<double>(particles_.size()) ? 1 : 0;
    }
    const auto wanted = static_cast<std::size_t>(std::ceil(kldBound(occupied, config_.kld_error, config_.kld_quantile)));
    const std::size_t n = std::clamp(wanted, config_.min_particles, config_.max_particles);
    // Low-variance (systematic) resampling: one random offset, n evenly spaced
    // pointers. Far less sampling noise than independent draws, so separate modes
    // (matching places) survive many more resamplings.
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::normal_distribution<double> jitter(0.0, 1.0);
    std::vector<Particle> next;
    next.reserve(n);
    const double step = total / static_cast<double>(n);
    double pointer = unit(rng_) * step;
    double cumulative = particles_.front().weight;
    std::size_t k = 0;
    for (std::size_t i = 0; i < n; ++i) {
        while (cumulative < pointer && k + 1 < particles_.size()) {
            cumulative += particles_[++k].weight;
        }
        pointer += step;
        Pose pose = particles_[k].pose;
        if (unit(rng_) < inject_fraction) {
            pose = randomFreePose();
            ++injected_;
        } else {
            // A little roughening keeps duplicates from staying identical.
            pose.x += 0.005 * jitter(rng_);
            pose.y += 0.005 * jitter(rng_);
            pose.theta = wrapAngle(pose.theta + 0.002 * jitter(rng_));
        }
        next.push_back({pose, 1.0 / static_cast<double>(n)});
    }
    particles_ = std::move(next);
}

ParticleEstimate ParticleFilter::estimate() const {
    ParticleEstimate result;
    if (particles_.empty()) {
        return result;
    }
    std::unordered_map<long long, double> bins;
    const auto bin = [](double v) { return static_cast<int>(std::floor(v / 0.5)); };
    for (const Particle& p : particles_) {
        bins[binKey(bin(p.pose.x), bin(p.pose.y), 0)] += p.weight;
    }
    // The 3 x 3 block of bins with the most weight.
    double best = -1.0;
    int bx = 0, by = 0;
    for (const auto& [key, weight] : bins) {
        (void)weight;
        const long long xy = key / 64;
        const int x = static_cast<int>(xy / 200003LL - 100000);
        const int y = static_cast<int>(xy % 200003LL - 100000);
        double block = 0.0;
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                const auto it = bins.find(binKey(x + dx, y + dy, 0));
                block += it == bins.end() ? 0.0 : it->second;
            }
        }
        if (block > best) {
            best = block;
            bx = x;
            by = y;
        }
    }
    double w = 0.0, mx = 0.0, my = 0.0, sc = 0.0, cc = 0.0;
    for (const Particle& p : particles_) {
        if (std::abs(bin(p.pose.x) - bx) <= 1 && std::abs(bin(p.pose.y) - by) <= 1) {
            w += p.weight;
            mx += p.weight * p.pose.x;
            my += p.weight * p.pose.y;
            sc += p.weight * std::sin(p.pose.theta);
            cc += p.weight * std::cos(p.pose.theta);
        }
    }
    mx /= w;
    my /= w;
    const double heading = std::atan2(sc, cc);
    double vxy = 0.0, vt = 0.0;
    for (const Particle& p : particles_) {
        if (std::abs(bin(p.pose.x) - bx) <= 1 && std::abs(bin(p.pose.y) - by) <= 1) {
            vxy += p.weight * ((p.pose.x - mx) * (p.pose.x - mx) + (p.pose.y - my) * (p.pose.y - my));
            const double e = wrapAngle(p.pose.theta - heading);
            vt += p.weight * e * e;
        }
    }
    result.pose = {mx, my, heading};
    result.share = w;
    result.position_std = std::sqrt(vxy / w);
    result.heading_std = std::sqrt(vt / w);
    return result;
}

std::vector<ParticleSample> runParticleFilter(const GridMap& map,
                                              const std::vector<TrajectorySample>& trajectory,
                                              const std::vector<LidarScan>& scans,
                                              const LidarConfig& lidar, const OdometryNoise& odometry,
                                              const ParticleFilterConfig& config, bool global_start,
                                              const std::vector<bool>& kidnapped) {
    std::vector<ParticleSample> samples;
    if (trajectory.empty()) {
        return samples;
    }
    ParticleFilter filter(map, config);
    // The same odometry noise sequence as runLocalization: three draws for the
    // initial error, then two per step.
    std::mt19937 rng(odometry.seed);
    std::normal_distribution<double> unit(0.0, 1.0);
    for (int i = 0; i < 3; ++i) {
        unit(rng);
    }
    const RobotState& s0 = trajectory.front().state;
    if (global_start) {
        filter.initializeGlobal();
    } else {
        filter.initializeAt({s0.x, s0.y, s0.theta}, 0.05, 0.05);
    }
    const std::vector<double> offsets = beamOffsets(lidar);
    const bool with_lidar = scans.size() == trajectory.size();
    for (std::size_t k = 0; k < trajectory.size(); ++k) {
        if (k > 0) {
            const OdometryIncrement truth = trueIncrement(trajectory[k - 1].state, trajectory[k].state);
            const double dt = trajectory[k].state.time - trajectory[k - 1].state.time;
            const OdometryIncrement commanded{trajectory[k].linear_velocity * dt,
                                              trajectory[k].angular_velocity * dt};
            const std::array<double, 2> var = odometryVariances(commanded, odometry);
            const double dn = std::sqrt(var[0]) * unit(rng);
            const double rn = std::sqrt(var[1]) * unit(rng);
            const bool jump = k < kidnapped.size() && kidnapped[k];
            const OdometryIncrement measured =
                jump ? OdometryIncrement{0.0, 0.0} : OdometryIncrement{truth.distance + dn, truth.rotation + rn};
            filter.predict(measured, jump ? std::array<double, 2>{0.0, 0.0} : var);
        }
        if (with_lidar) {
            filter.update(offsets, scans[k].ranges, scans[k].hits, lidar.max_range);
        }
        const RobotState& s = trajectory[k].state;
        samples.push_back({s.time, {s.x, s.y, s.theta}, filter.estimate(), filter.particles().size(),
                           filter.relocalizations(), filter.refinements()});
    }
    return samples;
}

}  // namespace amr
