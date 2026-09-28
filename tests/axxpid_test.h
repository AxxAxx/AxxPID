/**
 * @file  axxpid_test.h
 * @brief Minimal assert-based test harness and plant models for AxxPID.
 *
 * Deliberately framework-free: the library has no dependencies, and neither
 * do its tests. Each test file defines TESTS and calls AXXPID_TEST_MAIN.
 */

#ifndef AXXPID_TEST_H
#define AXXPID_TEST_H

#include <stdio.h>
#include <string.h>

#include "axxpid/axxpid.h"

/* -------------------------------------------------------------------------- */
/* Assertions                                                                 */
/* -------------------------------------------------------------------------- */

static int axxpid_test_failures = 0;
static int axxpid_test_checks = 0;
static const char *axxpid_test_current = "";

#define CHECK(cond)                                                            \
    do {                                                                       \
        axxpid_test_checks++;                                                  \
        if (!(cond)) {                                                         \
            axxpid_test_failures++;                                            \
            (void)printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);     \
        }                                                                      \
    } while (0)

#define CHECK_MSG(cond, ...)                                                   \
    do {                                                                       \
        axxpid_test_checks++;                                                  \
        if (!(cond)) {                                                         \
            axxpid_test_failures++;                                            \
            (void)printf("  FAIL %s:%d  %s -> ", __FILE__, __LINE__, #cond);   \
            (void)printf(__VA_ARGS__);                                         \
            (void)printf("\n");                                                \
        }                                                                      \
    } while (0)

/** @brief Absolute-tolerance float comparison. */
static int axxpid_test_near(axxpid_real_t a, axxpid_real_t b, axxpid_real_t tol)
{
    axxpid_real_t d = a - b;
    if (d < 0) {
        d = -d;
    }
    return d <= tol;
}

#define CHECK_NEAR(a, b, tol)                                                  \
    do {                                                                       \
        axxpid_test_checks++;                                                  \
        if (!axxpid_test_near((a), (b), (tol))) {                              \
            axxpid_test_failures++;                                            \
            (void)printf("  FAIL %s:%d  %s ~= %s  (%.6f vs %.6f, tol %.6f)\n", \
                         __FILE__, __LINE__, #a, #b, (double)(a), (double)(b), \
                         (double)(tol));                                       \
        }                                                                      \
    } while (0)

/* -------------------------------------------------------------------------- */
/* Test registration                                                          */
/* -------------------------------------------------------------------------- */

typedef struct {
    const char *name;
    void (*fn)(void);
} axxpid_test_case_t;

#define AXXPID_TEST_MAIN(suite_name, tests)                                    \
    int main(void)                                                             \
    {                                                                          \
        size_t i;                                                              \
        const size_t n = sizeof(tests) / sizeof((tests)[0]);                   \
        (void)printf("== %s ==\n", (suite_name));                              \
        for (i = 0; i < n; ++i) {                                              \
            const int before = axxpid_test_failures;                           \
            axxpid_test_current = (tests)[i].name;                             \
            (tests)[i].fn();                                                   \
            (void)printf("  %-44s %s\n", (tests)[i].name,                      \
                         (axxpid_test_failures == before) ? "ok" : "FAILED");  \
        }                                                                      \
        (void)printf("%s: %d checks, %d failures\n\n", (suite_name),           \
                     axxpid_test_checks, axxpid_test_failures);                \
        return (axxpid_test_failures == 0) ? 0 : 1;                            \
    }

#endif /* AXXPID_TEST_H */
