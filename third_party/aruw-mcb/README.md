# aruw-mcb pure pursuit (vendored)

ARUW's chassis pure pursuit, as a baseline for `tools/compare_pure_pursuit.cpp`.

Source: https://gitlab.com/aruw/controls/aruw-mcb at `1b274196c97489239cf49861615992f934c0f0fe`
(2026-07-04). GPL-3.0-or-later, see `COPYING`.

`src/` is copied unmodified:

| here | upstream |
| --- | --- |
| `src/aruwsrc/algorithms/auto_nav_path.{hpp,cpp}` | `aruw-mcb-project/src/aruwsrc/algorithms/` |
| `src/aruwsrc/algorithms/interpolate.hpp` | `aruw-mcb-project/src/aruwsrc/algorithms/` |
| `src/aruwsrc/control/chassis/chassis_auto_nav_controller.{hpp,cpp}` | `aruw-mcb-project/src/aruwsrc/control/chassis/` |
| `src/aruwsrc/control/chassis/beyblade_config.hpp` | `aruw-mcb-project/src/aruwsrc/control/chassis/` |
| `src/tap/algorithms/ramp.{hpp,cpp}` | `aruw-mcb-project/taproot/src/tap/algorithms/` |
| `src/tap/architecture/timeout.hpp` | `aruw-mcb-project/taproot/src/tap/architecture/` |

`shim/` is ours: host stand-ins for the parts of taproot and aruw-mcb that need the MCB
(CMSIS-DSP transforms, modm, drivers, the chassis subsystem, the cap bank, the clock). They
provide only what the files above use. The chassis records the commanded wheel speeds instead of
driving motors, `Transform` is a planar pose, and the clock is simulated time set by the caller
(`tap::arch::clock::setTimeMicroseconds`).

Build flags (the taproot files include `clock.hpp` and `math_user_utils.hpp` relative to
themselves):

```sh
-I third_party/aruw-mcb/shim -I third_party/aruw-mcb/src \
-iquote third_party/aruw-mcb/shim/tap/architecture -iquote third_party/aruw-mcb/shim/tap/algorithms
```

Two things to know when driving it:

- Build the path with `resetPath()` then `pushPoint()`, as the robot's commands do. A path built
  any other way never reports `hasChanged()`, the transition timeout never starts, and
  `calculateSetPoint` divides by `PATH_TRANSITION_TIME_MILLIS` (0).
- `runController` advances the speed ramp by `MAX_TRANSLATION_ACCELERATION` (1.0 m/s) on every
  call, not per second, so at any loop rate the speed reaches its target almost at once.
