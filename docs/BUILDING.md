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

---

## Repository layout

| Path | What it is |
|---|---|
| `include/axxpid/axxpid.h`, `src/axxpid.c` | The controller. The only mandatory files. |
| `include/axxpid/axxpid_tune.h`, `src/axxpid_tune.c` | Optional: tuning rules and the relay autotuner. Leave them out and the controller still builds. |
| `tests/` | Four suites, no framework. `axxpid_test.h` is the harness, `axxpid_plant.h` the simulated processes. |
| `examples/` | Four runnable desktop programs, plus STM32 and Arduino sketches (those two are not compiled by CI). |
| `docs/` | Reference material linked from the README. |

Build with the full warning set before calling anything done. The library is
clean at `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wdouble-promotion
-Wstrict-prototypes -Wmissing-prototypes -Werror` and should stay that way.
Test in both precisions: `-DAXXPID_USE_DOUBLE=1`.

---

## Invariants worth not breaking

- **No dependencies.** Not even `<math.h>` — `axxpid_sqrt` exists so the
  autotuner does not need libm. The only external symbol on a freestanding ARM
  build is compiler-emitted `memcpy`. Adding an include is a real decision, not
  a detail.
- **No allocation, no globals, no vendor HAL.** Every controller is a
  caller-allocated `axxpid_t`.
- **`ki` and `kd` are per second.** Everything integrates and differentiates
  against the measured elapsed time. Never fold a sample period into a gain.
- **The integral is stored in output units**, not as a raw error sum scaled at
  read time. That is what makes a runtime `ki` change bump-free.
- **Feed-forward is inside the saturation calculation.** Moving it outside
  reintroduces the windup-behind-feed-forward bug it was written to avoid.
- **Reverse action negates the error, never the gains**, so the getters return
  exactly what was set.
- **`axxpid_init` and `axxpid_init_config` always leave a usable controller**,
  even when they reject an argument. Every *other* entry point leaves state
  untouched on rejection. The asymmetry is deliberate: an uninitialised
  `axxpid_t` holds a garbage `ff_fn` pointer that the next update would call.

---

## Testing conventions

Expected values are derived by hand from the difference equations in
[CONTROL_LAW.md](CONTROL_LAW.md), or from the published tuning tables — never
recorded from a previous run. Two consequences when adding a test:

- Do not assert against the implementation's own expression.
  `CHECK_NEAR(g.ti, tu / 1.2f, ...)` passes even when both are wrong. Use the
  numeric literal.
- Keep tolerances just wide enough for error that is genuinely inherent. The
  autotuner's `Ku` is checked against the ~18% low that the describing-function
  approximation actually costs on the test plant, because a 30% band would not
  notice it drifting further.

Every bug found in review has a named regression test. When you fix a bug, add
one.

---

## Naming

Public API is everything declared in `include/axxpid/`. Everything else is
`static`, which in C is the whole of what "private" means. Internal helpers
still carry the `axxpid_` prefix, so that folding a source file into a unity
build cannot collide with another translation unit's `clamp` or `compute`.

Do **not** use a doubled underscore as a privacy marker. C reserves only
leading underscores, so `axxpid__clamp` is legal C — but C++ reserves any
identifier containing `__` anywhere, and these sources are compiled by C++
toolchains. A CI job rejects both forms.

---

## Things that look wrong but are not

- `AXXPID_UNLIMITED` is `1e30`, not `INFINITY`. Finite on purpose: no
  arithmetic inside the controller can then produce an infinity or a NaN from a
  limit alone, and the finiteness checks stay plain comparisons.
- The integral engagement band is **one-sided** (`error > band`). A symmetric
  version would dump the integral when the process is far *above* setpoint,
  which is exactly when a one-way actuator needs it.
- The derivative filter is **off by default**, so `kd` means what the equation
  says. The README tells users to enable it.
- `axxpid_get_i_term()` and `axxpid_get_integral()` differ on purpose: the
  first is the snapshot that went into the last output, the second is live
  state.
