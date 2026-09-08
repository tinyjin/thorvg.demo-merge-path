/* The half of skia's pathops test harness the case files call into. Each entry
   point records its two operands and its operator instead of solving them, so a
   corpus file becomes a table of fixtures rather than a test run. */
#ifndef _SKIACONF_EXTENDED_H_
#define _SKIACONF_EXTENDED_H_

#include "include/core/SkPath.h"
#include "include/pathops/SkPathOps.h"
#include <cstddef>

namespace skiatest { class Reporter; }
struct PathOpsThreadState;

struct TestDesc {
    void (*fun)(skiatest::Reporter*, const char* filename);
    const char* str;
};

namespace conf {
    //how skia itself rates the case: a plain one it expects to solve, a fail one
    //it only expects not to crash, a fuzz one it does not judge at all
    enum class Grade : uint8_t { Plain = 0, Check, Fail, Fuzz };
    void collect(const SkPath& a, const SkPath& b, SkPathOp op, const char* name, Grade grade);
}

inline bool testPathOp(skiatest::Reporter*, const SkPath& a, const SkPath& b, const SkPathOp op, const char* name)
{ conf::collect(a, b, op, name, conf::Grade::Plain); return true; }

inline bool testPathOpCheck(skiatest::Reporter*, const SkPath& a, const SkPath& b, const SkPathOp op, const char* name, bool)
{ conf::collect(a, b, op, name, conf::Grade::Check); return true; }

inline bool testPathOpFail(skiatest::Reporter*, const SkPath& a, const SkPath& b, const SkPathOp op, const char* name)
{ conf::collect(a, b, op, name, conf::Grade::Fail); return true; }

inline bool testPathOpFuzz(skiatest::Reporter*, const SkPath& a, const SkPath& b, const SkPathOp op, const char* name)
{ conf::collect(a, b, op, name, conf::Grade::Fuzz); return true; }

inline bool testSimplify(skiatest::Reporter*, const SkPath&, const char*) { return true; }
inline bool testSimplifyCheck(skiatest::Reporter*, const SkPath&, const char*, bool) { return true; }
inline bool testSimplifyFail(skiatest::Reporter*, const SkPath&, const char*) { return true; }
inline bool testSimplifyFuzz(skiatest::Reporter*, const SkPath&, const char*) { return true; }

inline void markTestFlakyForPathKit() {}
inline void initializeTests(skiatest::Reporter*, const char*) {}
inline void showOp(const SkPathOp) {}

inline void RunTestSet(skiatest::Reporter* reporter, TestDesc tests[], size_t count,
                       void (*)(skiatest::Reporter*, const char*),
                       void (*)(skiatest::Reporter*, const char*),
                       void (*)(skiatest::Reporter*, const char*), bool)
{
    for (size_t i = 0; i < count; ++i) tests[i].fun(reporter, tests[i].str);
}

#endif
