// Tests for the statistics behind the acceptance tests: special functions against
// closed forms and table values, and the intervals' coverage by simulation.
#include <cmath>
#include <cstddef>
#include <random>

#include "statistics.hpp"
#include "test_framework.hpp"

namespace {

double binomialCdf(std::size_t k, std::size_t n, double p) {
    double sum = 0.0;
    const auto nn = static_cast<double>(n);
    for (std::size_t i = 0; i <= k; ++i) {
        const auto ii = static_cast<double>(i);
        sum += std::exp(std::lgamma(nn + 1.0) - std::lgamma(ii + 1.0) - std::lgamma(nn - ii + 1.0) +
                        ii * std::log(p) + (nn - ii) * std::log1p(-p));
    }
    return sum;
}

void testSpecialFunctions() {
    // I_x(2, 3) = sum_{j=2..4} C(4, j) x^j (1 - x)^(4 - j); I_x(a, 1) = x^a; symmetry.
    CHECK_NEAR(amr::incompleteBeta(2.0, 3.0, 0.4), 0.5248, 1e-12);
    CHECK_NEAR(amr::incompleteBeta(3.7, 1.0, 0.6), std::pow(0.6, 3.7), 1e-12);
    CHECK_NEAR(amr::incompleteBeta(4.0, 9.0, 0.3) + amr::incompleteBeta(9.0, 4.0, 0.7), 1.0, 1e-12);
    CHECK_NEAR(amr::betaQuantile(0.5248, 2.0, 3.0), 0.4, 1e-10);
    // P(1, x) = 1 - e^-x; chi-square with 2 dof: quantile -2 ln(1 - p).
    CHECK_NEAR(amr::incompleteGamma(1.0, 2.5), 1.0 - std::exp(-2.5), 1e-13);
    CHECK_NEAR(amr::incompleteGamma(1.0, 40.0), 1.0, 1e-13);
    CHECK_NEAR(amr::chiSquareQuantile(0.9, 2.0), -2.0 * std::log(0.1), 1e-9);
    // Table values.
    CHECK_NEAR(amr::chiSquareQuantile(0.975, 3.0), 9.348404, 1e-5);
    CHECK_NEAR(amr::chiSquareQuantile(0.025, 3.0), 0.2157953, 1e-6);
    CHECK_NEAR(amr::chiSquareQuantile(0.95, 1.0), 3.841459, 1e-5);
    CHECK_NEAR(amr::chiSquareQuantile(0.99, 30.0), 50.89218, 1e-4);
    CHECK_NEAR(amr::normalQuantile(0.975), 1.959964, 1e-6);
    CHECK_NEAR(amr::normalQuantile(0.5), 0.0, 1e-12);
}

void testProportionBounds() {
    // No failures: the lower bound is (1 - c)^(1/n).
    CHECK_NEAR(amr::rateLowerBound(300, 300), std::pow(0.05, 1.0 / 300.0), 1e-10);
    CHECK_NEAR(amr::rateUpperBound(0, 10, 0.975), 1.0 - std::pow(0.025, 0.1), 1e-10);
    CHECK_EQ(amr::runsToDemonstrate(0.99), 299u);
    CHECK_EQ(amr::runsToDemonstrate(0.999), 2995u);
    // Clopper-Pearson by definition: P(X >= k | lower) = alpha/2, P(X <= k | upper) = alpha/2.
    const amr::Interval cp = amr::clopperPearson(5, 10);
    CHECK_NEAR(cp.lower, 0.187086, 1e-6);
    CHECK_NEAR(cp.upper, 0.812914, 1e-6);
    CHECK_NEAR(1.0 - binomialCdf(4, 10, cp.lower), 0.025, 1e-9);
    CHECK_NEAR(binomialCdf(5, 10, cp.upper), 0.025, 1e-9);
    const amr::Interval w = amr::wilson(5, 10);
    CHECK_NEAR(w.lower, 0.236593, 1e-5);
    CHECK_NEAR(w.upper, 0.763407, 1e-5);
    // Coverage by simulation: the exact interval never covers less than 95 %.
    std::mt19937 rng(3);
    for (const double p : {0.5, 0.9, 0.99}) {
        std::binomial_distribution<int> draw(60, p);
        int covered = 0;
        for (int i = 0; i < 2000; ++i) {
            const amr::Interval interval = amr::clopperPearson(static_cast<std::size_t>(draw(rng)), 60);
            covered += interval.lower <= p && p <= interval.upper ? 1 : 0;
        }
        CHECK(covered >= 1900 - 30);  // 95 % less three standard errors of the estimate
    }
}

void testNeesInterval() {
    // R runs of a consistent 3-state filter: mean NEES within the interval 95 % of the time.
    const amr::Interval one = amr::neesInterval(1, 3);
    CHECK_NEAR(one.lower, 0.2157953, 1e-6);
    CHECK_NEAR(one.upper, 9.348404, 1e-5);
    const amr::Interval many = amr::neesInterval(100, 3);
    CHECK(many.lower > 2.4 && many.lower < 2.6);
    CHECK(many.upper > 3.4 && many.upper < 3.6);
    std::mt19937 rng(5);
    std::chi_squared_distribution<double> nees(3.0);
    int inside = 0;
    for (int trial = 0; trial < 2000; ++trial) {
        double sum = 0.0;
        for (int r = 0; r < 100; ++r) {
            sum += nees(rng);
        }
        inside += sum / 100.0 >= many.lower && sum / 100.0 <= many.upper ? 1 : 0;
    }
    CHECK(inside >= 1870 && inside <= 1930);
}

void testMcNemar() {
    // Exact two-sided binomial p-values with p = 1/2.
    CHECK_NEAR(amr::mcnemarExact(0, 0), 1.0, 1e-12);
    CHECK_NEAR(amr::mcnemarExact(0, 6), 2.0 / 64.0, 1e-12);
    CHECK_NEAR(amr::mcnemarExact(4, 20), 2.0 * binomialCdf(4, 24, 0.5), 1e-12);
    CHECK_NEAR(amr::mcnemarExact(20, 4), amr::mcnemarExact(4, 20), 1e-12);
    CHECK_NEAR(amr::mcnemarExact(5, 5), 1.0, 1e-12);
}

}  // namespace

int main() {
    return testfw::runAll({
        {"Special functions", testSpecialFunctions},
        {"Success-rate bounds", testProportionBounds},
        {"NEES interval", testNeesInterval},
        {"McNemar's exact test", testMcNemar},
    });
}
