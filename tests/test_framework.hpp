// A deliberately tiny test harness so the core library builds and tests with
// nothing but a C++17 compiler (no network fetches, no package manager).
#pragma once

#include <cmath>
#include <cstring>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace httest {

struct TestCase {
  const char* name;
  std::function<void()> fn;
};

inline std::vector<TestCase>& registry() {
  static std::vector<TestCase> tests;
  return tests;
}

inline int& failureCount() {
  static int n = 0;
  return n;
}

struct Registrar {
  Registrar(const char* name, std::function<void()> fn) { registry().push_back({name, std::move(fn)}); }
};

inline void fail(const char* file, int line, const std::string& msg) {
  ++failureCount();
  std::cerr << "  " << file << ":" << line << ": " << msg << "\n";
}

inline int runAll(const char* filter) {
  int failedTests = 0;
  int ran = 0;
  for (const auto& t : registry()) {
    if (filter && !std::strstr(t.name, filter)) continue;
    ++ran;
    const int before = failureCount();
    t.fn();
    const bool ok = failureCount() == before;
    std::cout << (ok ? "[ PASS ] " : "[ FAIL ] ") << t.name << "\n";
    if (!ok) ++failedTests;
  }
  std::cout << "\n" << ran - failedTests << "/" << ran << " tests passed\n";
  return failedTests == 0 ? 0 : 1;
}

}  // namespace httest

#define HT_CAT_(a, b) a##b
#define HT_CAT(a, b) HT_CAT_(a, b)

#define TEST_CASE(name)                                                                   \
  static void HT_CAT(ht_test_, __LINE__)();                                               \
  static const ::httest::Registrar HT_CAT(ht_reg_, __LINE__)(name, &HT_CAT(ht_test_, __LINE__)); \
  static void HT_CAT(ht_test_, __LINE__)()

#define CHECK(cond)                                                     \
  do {                                                                  \
    if (!(cond)) ::httest::fail(__FILE__, __LINE__, "CHECK(" #cond ")"); \
  } while (0)

#define REQUIRE(cond)                                                     \
  do {                                                                    \
    if (!(cond)) {                                                        \
      ::httest::fail(__FILE__, __LINE__, "REQUIRE(" #cond ")");           \
      return;                                                             \
    }                                                                     \
  } while (0)

#define CHECK_EQ(a, b)                                                                  \
  do {                                                                                  \
    if (!((a) == (b))) ::httest::fail(__FILE__, __LINE__, "CHECK_EQ(" #a ", " #b ")"); \
  } while (0)

#define CHECK_NEAR(a, b, eps)                                                     \
  do {                                                                            \
    const double ht_a_ = static_cast<double>(a);                                  \
    const double ht_b_ = static_cast<double>(b);                                  \
    if (!(std::fabs(ht_a_ - ht_b_) <= (eps))) {                                   \
      std::ostringstream ht_os_;                                                  \
      ht_os_ << "CHECK_NEAR(" #a ", " #b "): " << ht_a_ << " vs " << ht_b_;       \
      ::httest::fail(__FILE__, __LINE__, ht_os_.str());                           \
    }                                                                             \
  } while (0)
