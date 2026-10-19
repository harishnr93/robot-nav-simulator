// Statistics for acceptance tests: confidence bounds on success rates, consistency
// bounds on the NEES, and an exact test for paired before/after comparisons.
//
// Success rates. k successes in n independent runs. The Clopper-Pearson interval is
// exact (it never covers less than the stated confidence): its lower bound for
// confidence c (one-sided) is the (1 - c) quantile of Beta(k, n - k + 1), its upper
// bound the c quantile of Beta(k + 1, n - k). With no failures the lower bound is
// (1 - c)^(1/n): 300 clean runs show a rate of at least 99.0 % at 95 % confidence.
// The Wilson score interval is the usual approximate alternative.
//
// NEES. For a consistent filter, the NEES e^T P^-1 e of a d-dimensional estimate is
// chi-square with d degrees of freedom. Samples from one run are correlated in time,
// so only independent runs count: the NEES of R runs at the same time step, summed,
// is chi-square with R d degrees of freedom, and their mean lies in
// [chi2(alpha/2, R d), chi2(1 - alpha/2, R d)] / R with probability 1 - alpha.
//
// Paired comparisons. Two variants run on the same scenarios: b runs only the first
// variant fails, c runs only the second fails. Under "no difference" each discordant
// run is equally likely to go either way, so McNemar's exact test is a binomial test
// of b against b + c with p = 1/2.
#pragma once

#include <cstddef>

namespace amr {

/// Regularized incomplete beta function I_x(a, b), a, b > 0, x in [0, 1].
double incompleteBeta(double a, double b, double x);

/// x with I_x(a, b) = p (the quantile of Beta(a, b)).
double betaQuantile(double p, double a, double b);

/// Regularized lower incomplete gamma function P(a, x).
double incompleteGamma(double a, double x);

/// The p quantile of the chi-square distribution with `dof` degrees of freedom.
double chiSquareQuantile(double p, double dof);

/// The p quantile of the standard normal distribution.
double normalQuantile(double p);

struct Interval {
    double lower = 0.0;
    double upper = 1.0;
};

/// Clopper-Pearson interval for k successes in n runs: two-sided with coverage
/// `confidence` (each side 1 - (1 - confidence) / 2).
Interval clopperPearson(std::size_t k, std::size_t n, double confidence = 0.95);

/// One-sided Clopper-Pearson bounds: the rate is at least `lower` (at most `upper`)
/// with probability `confidence`.
double rateLowerBound(std::size_t k, std::size_t n, double confidence = 0.95);
double rateUpperBound(std::size_t k, std::size_t n, double confidence = 0.95);

/// Wilson score interval (two-sided).
Interval wilson(std::size_t k, std::size_t n, double confidence = 0.95);

/// Runs without a failure needed to show a rate of at least `rate` at `confidence`.
std::size_t runsToDemonstrate(double rate, double confidence = 0.95);

/// Interval for the mean NEES of `runs` independent runs of a consistent filter
/// with a `dimension`-dimensional state, two-sided with coverage `confidence`.
Interval neesInterval(std::size_t runs, std::size_t dimension, double confidence = 0.95);

/// McNemar's exact test (two-sided p-value): b runs only the first variant fails, c
/// runs only the second fails.
double mcnemarExact(std::size_t b, std::size_t c);

}  // namespace amr
