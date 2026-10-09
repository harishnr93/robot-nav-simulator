// Minimal assertion-based test framework (no external dependencies).
//
// CHECK*   records a failure and continues the test case.
// REQUIRE  records a failure and returns from the test case (use it to guard
//          code that would otherwise index out of range).
#pragma once

#include <cmath>
#include <exception>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace testfw {

inline int& failureCount() {
    static int failures = 0;
    return failures;
}

inline void reportFailure(const char* file, int line, const std::string& message) {
    ++failureCount();
    std::cerr << "    " << file << ':' << line << ": " << message << '\n';
}

struct TestCase {
    std::string name;
    std::function<void()> body;
};

inline int runAll(const std::vector<TestCase>& tests) {
    int failed = 0;
    for (const TestCase& test : tests) {
        const int before = failureCount();
        try {
            test.body();
        } catch (const std::exception& e) {
            reportFailure(__FILE__, __LINE__, std::string("unexpected exception: ") + e.what());
        }
        const bool passed = failureCount() == before;
        std::cout << (passed ? "[PASS] " : "[FAIL] ") << test.name << std::endl;
        if (!passed) {
            ++failed;
        }
    }
    const auto total = static_cast<int>(tests.size());
    std::cout << (total - failed) << '/' << total << " test cases passed" << std::endl;
    return failed == 0 ? 0 : 1;
}

}  // namespace testfw

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            ::testfw::reportFailure(__FILE__, __LINE__, "CHECK(" #cond ") failed"); \
        }                                                                        \
    } while (false)

#define REQUIRE(cond)                                                              \
    do {                                                                           \
        if (!(cond)) {                                                             \
            ::testfw::reportFailure(__FILE__, __LINE__, "REQUIRE(" #cond ") failed"); \
            return;                                                                \
        }                                                                          \
    } while (false)

#define CHECK_EQ(actual, expected)                                                 \
    do {                                                                           \
        const auto& check_actual_ = (actual);                                      \
        const auto& check_expected_ = (expected);                                  \
        if (!(check_actual_ == check_expected_)) {                                 \
            std::ostringstream check_msg_;                                         \
            check_msg_ << "CHECK_EQ(" #actual ", " #expected "): got "            \
                       << check_actual_ << ", expected " << check_expected_;      \
            ::testfw::reportFailure(__FILE__, __LINE__, check_msg_.str());         \
        }                                                                          \
    } while (false)

#define CHECK_NEAR(actual, expected, tolerance)                                    \
    do {                                                                           \
        const double check_actual_ = (actual);                                     \
        const double check_expected_ = (expected);                                 \
        if (!(std::fabs(check_actual_ - check_expected_) <= (tolerance))) {        \
            std::ostringstream check_msg_;                                         \
            check_msg_ << "CHECK_NEAR(" #actual ", " #expected "): got "          \
                       << check_actual_ << ", expected " << check_expected_       \
                       << " +/- " << (tolerance);                                  \
            ::testfw::reportFailure(__FILE__, __LINE__, check_msg_.str());         \
        }                                                                          \
    } while (false)
