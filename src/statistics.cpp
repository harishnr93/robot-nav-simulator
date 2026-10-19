#include "statistics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace amr {

namespace {

/// Continued fraction for the incomplete beta function (modified Lentz).
double betaFraction(double a, double b, double x) {
    constexpr double kTiny = 1e-300;
    const double qab = a + b, qap = a + 1.0, qam = a - 1.0;
    double c = 1.0;
    double d = 1.0 - qab * x / qap;
    d = 1.0 / (std::fabs(d) < kTiny ? kTiny : d);
    double h = d;
    for (int m = 1; m <= 10000; ++m) {
        const double m2 = 2.0 * m;
        double aa = m * (b - m) * x / ((qam + m2) * (a + m2));
        d = 1.0 + aa * d;
        d = 1.0 / (std::fabs(d) < kTiny ? kTiny : d);
        c = 1.0 + aa / c;
        c = std::fabs(c) < kTiny ? kTiny : c;
        h *= d * c;
        aa = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2));
        d = 1.0 + aa * d;
        d = 1.0 / (std::fabs(d) < kTiny ? kTiny : d);
        c = 1.0 + aa / c;
        c = std::fabs(c) < kTiny ? kTiny : c;
        const double delta = d * c;
        h *= delta;
        if (std::fabs(delta - 1.0) < 1e-15) {
            break;
        }
    }
    return h;
}

/// Bisection for an increasing function on [lo, hi].
template <typename F>
double invert(F f, double target, double lo, double hi) {
    for (int i = 0; i < 200; ++i) {
        const double mid = 0.5 * (lo + hi);
        (f(mid) < target ? lo : hi) = mid;
        if (hi - lo < 1e-14 * std::max(1.0, std::fabs(mid))) {
            break;
        }
    }
    return 0.5 * (lo + hi);
}

}  // namespace

double incompleteBeta(double a, double b, double x) {
    if (x <= 0.0) {
        return 0.0;
    }
    if (x >= 1.0) {
        return 1.0;
    }
    const double front =
        std::exp(std::lgamma(a + b) - std::lgamma(a) - std::lgamma(b) + a * std::log(x) + b * std::log1p(-x));
    // The fraction converges fast for x < (a + 1) / (a + b + 2); use the symmetry otherwise.
    if (x < (a + 1.0) / (a + b + 2.0)) {
        return front * betaFraction(a, b, x) / a;
    }
    return 1.0 - front * betaFraction(b, a, 1.0 - x) / b;
}

double betaQuantile(double p, double a, double b) {
    if (p <= 0.0) {
        return 0.0;
    }
    if (p >= 1.0) {
        return 1.0;
    }
    return invert([&](double x) { return incompleteBeta(a, b, x); }, p, 0.0, 1.0);
}

double incompleteGamma(double a, double x) {
    if (x <= 0.0) {
        return 0.0;
    }
    const double log_front = -x + a * std::log(x) - std::lgamma(a);
    if (x < a + 1.0) {  // series
        double sum = 1.0 / a, term = sum;
        for (int n = 1; n < 100000; ++n) {
            term *= x / (a + n);
            sum += term;
            if (std::fabs(term) < std::fabs(sum) * 1e-16) {
                break;
            }
        }
        return sum * std::exp(log_front);
    }
    // continued fraction for Q (modified Lentz)
    constexpr double kTiny = 1e-300;
    double b = x + 1.0 - a, c = 1.0 / kTiny, d = 1.0 / b, h = d;
    for (int i = 1; i < 100000; ++i) {
        const double an = -i * (i - a);
        b += 2.0;
        d = an * d + b;
        d = std::fabs(d) < kTiny ? kTiny : d;
        c = b + an / c;
        c = std::fabs(c) < kTiny ? kTiny : c;
        d = 1.0 / d;
        const double delta = d * c;
        h *= delta;
        if (std::fabs(delta - 1.0) < 1e-16) {
            break;
        }
    }
    return 1.0 - std::exp(log_front) * h;
}

double chiSquareQuantile(double p, double dof) {
    const double hi = dof + 20.0 * std::sqrt(2.0 * dof) + 100.0;
    return invert([&](double x) { return incompleteGamma(0.5 * dof, 0.5 * x); }, p, 0.0, hi);
}

double normalQuantile(double p) {
    return invert([](double z) { return 0.5 * std::erfc(-z / std::sqrt(2.0)); }, p, -40.0, 40.0);
}

double rateLowerBound(std::size_t k, std::size_t n, double confidence) {
    if (k == 0 || n == 0) {
        return 0.0;
    }
    return betaQuantile(1.0 - confidence, static_cast<double>(k), static_cast<double>(n - k + 1));
}

double rateUpperBound(std::size_t k, std::size_t n, double confidence) {
    if (k >= n) {
        return 1.0;
    }
    return betaQuantile(confidence, static_cast<double>(k + 1), static_cast<double>(n - k));
}

Interval clopperPearson(std::size_t k, std::size_t n, double confidence) {
    const double side = 1.0 - (1.0 - confidence) / 2.0;
    return {rateLowerBound(k, n, side), rateUpperBound(k, n, side)};
}

Interval wilson(std::size_t k, std::size_t n, double confidence) {
    if (n == 0) {
        return {};
    }
    const double z = normalQuantile(1.0 - (1.0 - confidence) / 2.0);
    const double nn = static_cast<double>(n), p = static_cast<double>(k) / nn;
    const double centre = (p + z * z / (2.0 * nn)) / (1.0 + z * z / nn);
    const double half = z * std::sqrt(p * (1.0 - p) / nn + z * z / (4.0 * nn * nn)) / (1.0 + z * z / nn);
    return {std::max(0.0, centre - half), std::min(1.0, centre + half)};
}

std::size_t runsToDemonstrate(double rate, double confidence) {
    if (rate <= 0.0) {
        return 1;
    }
    if (rate >= 1.0) {
        return std::numeric_limits<std::size_t>::max();
    }
    return static_cast<std::size_t>(std::ceil(std::log(1.0 - confidence) / std::log(rate) - 1e-12));
}

Interval neesInterval(std::size_t runs, std::size_t dimension, double confidence) {
    const double dof = static_cast<double>(runs * dimension);
    const double alpha = 1.0 - confidence;
    const double r = static_cast<double>(runs);
    return {chiSquareQuantile(alpha / 2.0, dof) / r, chiSquareQuantile(1.0 - alpha / 2.0, dof) / r};
}

double mcnemarExact(std::size_t b, std::size_t c) {
    const std::size_t n = b + c;
    if (n == 0) {
        return 1.0;
    }
    const std::size_t k = std::min(b, c);
    // P(X <= k) for X ~ Binomial(n, 1/2), doubled.
    double tail = 0.0;
    for (std::size_t i = 0; i <= k; ++i) {
        tail += std::exp(std::lgamma(static_cast<double>(n) + 1.0) - std::lgamma(static_cast<double>(i) + 1.0) -
                         std::lgamma(static_cast<double>(n - i) + 1.0) - static_cast<double>(n) * std::log(2.0));
    }
    return std::min(1.0, 2.0 * tail);
}

}  // namespace amr
