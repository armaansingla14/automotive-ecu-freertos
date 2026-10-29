// Minimal self-contained unit test framework (no external dependencies so
// the project builds offline and in CI containers without network access).
#pragma once
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

namespace testfw {

struct TestCase {
  const char* name;
  std::function<void()> fn;
};

inline std::vector<TestCase>& registry() {
  static std::vector<TestCase> r;
  return r;
}

struct Registrar {
  Registrar(const char* name, std::function<void()> fn) { registry().push_back({name, std::move(fn)}); }
};

struct Failure {
  std::string msg;
};

inline int& failures_in_test() {
  static int f = 0;
  return f;
}

inline void report(const char* file, int line, const std::string& msg) {
  std::printf("    %s:%d: %s\n", file, line, msg.c_str());
  ++failures_in_test();
}

// printable form for failure messages (promotes uint8_t to int)
template <typename T>
auto show(const T& v) -> decltype(+v) {
  return +v;
}
inline const std::string& show(const std::string& s) { return s; }

inline int run_all(int argc, char** argv) {
  const char* filter = argc > 1 ? argv[1] : nullptr;
  int passed = 0, failed = 0;
  for (const auto& tc : registry()) {
    if (filter && !std::strstr(tc.name, filter)) continue;
    failures_in_test() = 0;
    try {
      tc.fn();
    } catch (const Failure&) {
    } catch (const std::exception& e) {
      report(__FILE__, __LINE__, std::string("unexpected exception: ") + e.what());
    }
    if (failures_in_test() == 0) {
      ++passed;
    } else {
      ++failed;
      std::printf("[  FAILED  ] %s\n", tc.name);
    }
  }
  std::printf("[==========] %d tests, %d passed, %d failed\n", passed + failed, passed, failed);
  return failed == 0 ? 0 : 1;
}

}  // namespace testfw

#define TF_CAT2(a, b) a##b
#define TF_CAT(a, b) TF_CAT2(a, b)
#define TEST(suite, name)                                                                    \
  static void TF_CAT(test_, TF_CAT(suite, TF_CAT(_, name)))();                               \
  static testfw::Registrar TF_CAT(reg_, TF_CAT(suite, TF_CAT(_, name)))(                     \
      #suite "." #name, &TF_CAT(test_, TF_CAT(suite, TF_CAT(_, name))));                     \
  static void TF_CAT(test_, TF_CAT(suite, TF_CAT(_, name)))()

#define CHECK(cond)                                                    \
  do {                                                                 \
    if (!(cond)) testfw::report(__FILE__, __LINE__, "CHECK(" #cond ")"); \
  } while (0)

#define REQUIRE(cond)                                                    \
  do {                                                                   \
    if (!(cond)) {                                                       \
      testfw::report(__FILE__, __LINE__, "REQUIRE(" #cond ")");          \
      throw testfw::Failure{};                                           \
    }                                                                    \
  } while (0)

#define CHECK_EQ(a, b)                                                                     \
  do {                                                                                     \
    const auto _va = (a);                                                                  \
    const auto _vb = (b);                                                                  \
    if (!(_va == _vb)) {                                                                   \
      std::ostringstream _os;                                                              \
      _os << "CHECK_EQ(" #a ", " #b ") got " << testfw::show(_va) << " vs " << testfw::show(_vb);                   \
      testfw::report(__FILE__, __LINE__, _os.str());                                       \
    }                                                                                      \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                              \
  do {                                                                                     \
    const double _va = (a), _vb = (b);                                                     \
    if (std::fabs(_va - _vb) > (tol)) {                                                    \
      std::ostringstream _os;                                                              \
      _os << "CHECK_NEAR(" #a ", " #b ") got " << _va << " vs " << _vb;                   \
      testfw::report(__FILE__, __LINE__, _os.str());                                       \
    }                                                                                      \
  } while (0)

#define TEST_MAIN() \
  int main(int argc, char** argv) { return testfw::run_all(argc, argv); }
