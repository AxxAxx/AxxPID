# Building, testing and contributing

## Building and testing

```bash
cmake -B build -DAXXPID_STRICT=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

Four suites:

| Suite | Covers |
|---|---|
| `test_core` | Lifecycle, argument validation, NULL safety on every entry point, NaN/Inf rejection, tick wraparound, scheduling. |
| `test_features` | One test per control-law feature, checked against hand-derived arithmetic, plus a regression for every defect found in review. |
| `test_closedloop` | Closed-loop runs against a first-order-plus-dead-time plant: settling, overshoot, disturbance rejection, jitter tolerance, the AxxSolder profile. |
| `test_tune` | Rule tables against the published coefficients; the relay autotuner against a plant whose true `Ku` and `Tu` are computed numerically in the test. |

808 assertions in total, in both `float` and `double`. Expected values are
derived from the difference equations above or from the published tuning
tables, not recorded from a previous run, so they catch a change in behaviour
rather than merely pinning it.

They are also checked by mutation testing: twenty deliberately broken copies
of the library — an inverted filter constant, a dropped anti-windup condition,
a missing sign — are built and run against the suite. Nineteen are caught. The
one that is not changes a square root by 0.04%, which is far below anything
that matters here. A test suite that no broken version can fail is not a test
suite, and counting assertions does not tell you which you have.

CI builds with GCC and Clang on Linux, macOS and Windows, in both `float` and
`double`, against C99, C11 and C17, from C++, and cross-compiles for Cortex-M4F
and Cortex-M0+ with `-Wall -Wextra -Wpedantic -Wshadow -Wconversion
-Wdouble-promotion -Werror`.

Examples build with the tests and run standalone:

```bash
./build/01_minimal          # a PI loop on a tank
./build/02_feedforward      # three kinds of feed-forward, side by side
./build/03_autotune         # relay autotune, then control with the result
./build/04_soldering_iron   # the AxxSolder configuration, explained
```

---

## Continuous integration

CI builds with GCC and Clang on Linux, macOS and Windows, in both `float` and
`double`, against C99, C11 and C17, from C++, and cross-compiles for Cortex-M4F
and Cortex-M0+ with `-Wall -Wextra -Wpedantic -Wshadow -Wconversion
-Wdouble-promotion -Werror`. A separate job rejects any identifier reserved to
the implementation — a leading underscore, or a doubled underscore anywhere.

## Mutation testing

Counting assertions tells you very little. What tells you something is
breaking the library on purpose and checking that the suite notices.

Twenty deliberately broken copies — an inverted filter constant, a dropped
anti-windup condition, a missing sign, a collapsed time accumulator — are
built and run against the suite. Nineteen are caught. The one that is not
changes a square root by 0.04%, far below anything that matters here.

If you add a feature, break it once on purpose and confirm the suite fails.
