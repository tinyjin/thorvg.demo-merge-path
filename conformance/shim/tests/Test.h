/* A stand-in for skia's unit test framework, holding only what the pathops case
   files touch. Including the real one would drag in the whole test binary; all we
   want out of those files is the geometry, so DEF_TEST becomes a registration and
   the assertions become nothing. */
#ifndef _SKIACONF_TEST_H_
#define _SKIACONF_TEST_H_

#include <cstdio>

namespace skiatest {
    class Reporter
    {
    public:
        bool allowExtendedTest() const { return false; }
        bool verbose() const { return false; }
        void bumpTestCount() {}
    };
}

namespace conf {
    //every DEF_TEST in a corpus file lands here, and main() walks them
    void enroll(const char* name, void (*fn)(skiatest::Reporter*));
}

#define DEF_TEST(name, reporter)                                                  \
    static void conf_body_##name(skiatest::Reporter* reporter);                   \
    static const int conf_slot_##name = (conf::enroll(#name, conf_body_##name), 0);\
    static void conf_body_##name(skiatest::Reporter* reporter)

#define DEF_TEST_DISABLED(name, reporter)                                         \
    static void conf_unused_##name(skiatest::Reporter* reporter)

#define REPORTER_ASSERT(r, cond, ...) do { (void)(r); (void)!!(cond); } while (0)
#define ERRORF(r, ...) do { (void)(r); } while (0)
#define INFOF(r, ...) do { (void)(r); } while (0)

#endif
