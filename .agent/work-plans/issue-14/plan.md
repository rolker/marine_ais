# Plan: Build on ROS 2 Rolling: port ais_layer and marine_ais_tools, add a rolling branch

## Issue

https://github.com/rolker/marine_ais/issues/14

## Context

`marine_ais` (three packages: `marine_ais_msgs`, `marine_ais_tools`, `ais_layer`) is at the tip of `jazzy` (c821363); `origin/rolling` exists at the same commit and `feature/issue-14` starts from it. Nothing has been built on Rolling. The issue lists four known edits; the build will probably find more. The repo has no CI and no README. Rolling and Lyrical are installed here; Jazzy is not (checkable only by CI). The PR targets `rolling`, body `Part of #14`. The backport to `jazzy` follows outside this plan (host, by hand: cherry-pick, PR to `jazzy`, CI as its check).

Owner decisions (fixed): CI option B; `.agent/work-plans/issue-14/` records ride the branch and PR.

## Approach

Each fix is its own commit so the backport can cherry-pick it.

1. **Baseline build** — in the worktree, source its `env.sh`, run its `build.sh` and `test.sh` for all three packages on Rolling. Record the failures that appear (the four known ones and any others). The Rolling underlay (`/home/roland/project11/rolling/underlay`, nav2 and geodesy from source) must have finished installing; if its build is still running, build `marine_ais_msgs` and `marine_ais_tools` first and `ais_layer` after.
2. **`marine_ais_tools/setup.py`** — replace `tests_require=['pytest']` with `extras_require={'test': ['pytest']}`. Commit. Verify a Python test (e.g. `test_covariance.py`) is actually collected and runs on Rolling.
3. **`ais_parser.py:329`** — `self.get_logger().warn(s)` becomes `.warning(s)` (already confirmed an rclpy logger, `self.get_logger()`). Grep the package for other deprecated calls (`.warn(`, `tf_transformations` use, etc.) and fix any. Commit.
4. **`ais_layer/src/ais_layer.cpp:20`** — `tf2/utils.h` becomes `tf2/utils.hpp`. `utils.hpp` exists here on Rolling and Lyrical; its presence on Jazzy is not checkable locally and is confirmed by the CI Jazzy leg (it is in tf2 0.36 as far as I know, unverified). Commit.
5. **`ais_layer/CMakeLists.txt` geodesy** — Rolling's `geodesy` (source at `underlay/src/geographic_info/geodesy`, f70b81a) exports no CMake target: it does `ament_export_libraries(geoconv)` and `ament_export_include_directories(include)` only. So the replacement for `ament_target_dependencies(... geodesy)` is `target_include_directories(... ${geodesy_INCLUDE_DIRS})` plus linking `${geodesy_LIBRARIES}`. The code uses only `geodesy/ecef.h` and `geodesy/wgs84.h` (header-style conversions), so the library may be unneeded; test first whether `${geodesy_INCLUDE_DIRS}` alone links. Use the form that builds on Rolling and also reads sensibly on Jazzy (CI decides); do not rely on `geodesy::` targets unless the build shows they exist. Remove the `ament_target_dependencies` block. Other deps already use targets. Commit.
6. **Further fixes** — whatever step 1 and the full rebuild turn up (e.g. `nav2_util` test target names, nav2 header moves, lint failures from newer ament linters), one commit each, in Jazzy-compatible forms.
7. **CI: `.github/workflows/ci.yml`** (own commit, standalone so it cherry-picks to `jazzy`). Model on marine_control's `ci.yml` (container `ros:<distro>-ros-core`, rosdep, colcon build, test, test-result). Matrix `distro: [jazzy, rolling]` with `fail-fast: false`.
   - Jazzy leg: `colcon build` and `colcon test` all three packages. `rosdep` pulls `nav2_costmap_2d`, `nav2_util`, `geodesy` as released packages.
   - Rolling leg: `--packages-up-to marine_ais_msgs marine_ais_tools` build and test only (`ais_layer` excluded; navigation2 and geodesy have no Rolling release, so rosdep would fail on it). Implemented as a matrix field (e.g. `packages`) so one job definition serves both. With `--packages-skip ais_layer` the rosdep step must also skip it (`rosdep install --skip-keys` or install from the selected paths only); check this works.
   - Triggers: `push` and `pull_request` on `branches: [rolling, jazzy]`. On `rolling` the matrix runs as above. On `jazzy` the Rolling leg should not run; guard it with a job `if` (or matrix `exclude` on `github.ref`/`github.base_ref`) so a PR or push to `jazzy` runs the Jazzy leg only (Jazzy leg, all three packages). That makes the file behave sensibly when cherry-picked to `jazzy` and gives the backport PR its check. If the guard proves awkward, the fallback is to leave the Rolling leg running on `jazzy` too (it still passes); say so in the PR.
   - Header comment states: the triggers; what each leg builds; that `ais_layer` on Rolling is not covered by CI and is checked by hand locally; and that option A (build Nav2 from source in the Rolling leg) is the follow-up if it proves cheap.
   - The workflow cannot be run here; validate with `yamllint` or `actionlint` if installed, and by reading it against marine_control's working file. The first real run is the PR itself.
8. **Final verification** — clean build and test of all three packages on Rolling (with the underlay), including `ais_layer` and its gtests, via the worktree's `build.sh` and `test.sh`. This is the hand check of the CI gap; paste the result summary in the PR.
9. **PR to `rolling`** (host pushes) — body `Part of #14`; states what was and was not checked: Rolling all three packages locally; Jazzy only by CI; `ais_layer` on Rolling by hand only, not in CI; the backport to `jazzy` follows outside this PR (cherry-pick, PR to `jazzy`, CI as its check). Cost write-up goes to the issue as a comment, like marine_control#13.

## Files to Change

| File | Change |
|------|--------|
| `marine_ais_tools/setup.py` | `tests_require` to `extras_require={'test': ['pytest']}` |
| `marine_ais_tools/marine_ais_tools/ais_parser.py` | `.warn(` to `.warning(`, plus any other deprecated calls found |
| `ais_layer/src/ais_layer.cpp` | `tf2/utils.h` to `tf2/utils.hpp` |
| `ais_layer/CMakeLists.txt` | drop `ament_target_dependencies`; geodesy via include dirs/libraries variables |
| `.github/workflows/ci.yml` | new: Jazzy full, Rolling msgs+tools, branches `rolling` and `jazzy` |
| `.agent/work-plans/issue-14/` | plan and progress records, carried in the PR |
| others as the build finds | one commit each |

## Prior Art

rolker/marine_control PR #14 and its `ci.yml` (working Jazzy + Rolling matrix): taken for the workflow shape and the lessons (`${<msgs>_TARGETS}` rather than aggregate targets, `extras_require`, `warning`, one commit per fix). Rejected from it: building every package on Rolling, because `ais_layer` needs Nav2 and geodesy from source. Option A (Nav2 source build in CI) was considered and left as a follow-up by the owner's decision.

## Serves

None (project has no written goals); no project design document. Part of rolker/unh_marine_autonomy#415.

## Design Picture

No change to the design picture; the project has no design document.

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Test what breaks | Rolling leg of CI fails without the setup.py/warn fixes; the `ais_layer` fixes are guarded by the Jazzy leg and the local Rolling build, with the Rolling CI hole stated. |
| Verify before claiming | Real Rolling build of all three packages; Jazzy claims limited to what CI shows. `tf2/utils.hpp` on Jazzy and geodesy linkage on Jazzy are unverified until CI runs. |
| Small steps | One commit per fix; CI commit standalone. |
| Leave in a project only what it chose to carry | Owner chose to carry the `.agent/work-plans/issue-14/` records. |
| Know whether it works | Cost write-up comment on the issue. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 Worktree isolation | Yes | All work in the issue worktree. |
| 0019 repo_pin | Yes | PR base `rolling`; `jazzy` via the host's backport. |
| 0013 progress entries | Yes | `## Plan Authored` entry cites the plan commit. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Add `.github/workflows/` | triggers must name `rolling` and `jazzy`; workflow must work when cherry-picked | Yes (step 7) |
| Repo has no README | nothing to update for supported distros | Yes (none exists; not adding one) |
| Code fixes | backport to `jazzy` | Outside this plan (host, by hand) |
| `ais_layer` not in Rolling CI | header comment and PR text state the gap | Yes |

## Open Questions

- Does Jazzy's `geodesy` behave the same with `${geodesy_INCLUDE_DIRS}`/`${geodesy_LIBRARIES}`? Only CI can say; if it fails on Jazzy the fix may need a small distro-neutral variant.
- Is the `if`-guarded Rolling leg on `jazzy` acceptable, or should the Rolling leg simply also run on `jazzy`? Default: guard it off on `jazzy`.
- Whether the underlay install finishes in time for the local `ais_layer` check on Rolling.

## Estimated Scope

Single PR to `rolling` (about 5 to 7 small commits); the `jazzy` backport is a second PR outside this plan.
