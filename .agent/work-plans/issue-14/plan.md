# Plan: Build on ROS 2 Rolling: port ais_layer and marine_ais_tools, add a rolling branch

## Issue

https://github.com/rolker/marine_ais/issues/14

## Context

`marine_ais` (three packages: `marine_ais_msgs`, `marine_ais_tools`, `ais_layer`) is at the tip of `jazzy` (c821363); `origin/rolling` exists at the same commit and `feature/issue-14` starts from it. Nothing has been built on Rolling. The issue lists four known edits; the build will probably find more. The repo has no CI and no README. Rolling is installed here (Ubuntu 26.04); Jazzy is not, so Jazzy is checkable only by CI. The PR targets `rolling`, body `Part of #14`. The backport to `jazzy` follows outside this plan (host, by hand: cherry-pick, PR to `jazzy`, CI as its check).

Owner decisions (fixed): CI option B (Jazzy leg all three packages; Rolling leg `marine_ais_msgs` and `marine_ais_tools` only; the `ais_layer` gap on Rolling stated); `.agent/work-plans/issue-14/` records ride the branch and PR to `rolling`; the backport is a hand cherry-pick, so the CI commit must behave sensibly when cherry-picked onto `jazzy`.

Facts established by plan review and the host (these shape the plan):

- `ais_layer.cpp` includes `geodesy/ecef.h` and uses `geodesy::ECEFPoint`. ECEF support is in geographic_info 1.0.7 and later. rosdistro has geographic_info `1.0.6-2` for Jazzy, whose `geodesy` has only `utm.h` and `wgs84.h`; so a rosdep-installed `geodesy` on Jazzy cannot build `ais_layer`. Local builds hid this because `project11.manifest.yaml` builds geographic_info from source for every distro.
- `geodesy` exports no CMake target on 1.0.6 or 1.0.7: only `ament_export_libraries(geoconv)` and `ament_export_include_directories(include)`. `geoconv` is a STATIC library holding the `ECEFPoint(GeoPoint)` definition (`src/conv/ecef_conversions.cpp`), so it must be linked. Local underlay: geographic_info `1.0.7-1-gf70b81a`, with `libgeoconv.a` installed.
- Rolling: rosdistro lists geographic_info `1.0.7-4` (so a `geodesy` release exists), but on Ubuntu 26.04 (resolute) `ros-rolling-geodesy` has no apt candidate; whether it installs in a CI container depends on the container's Ubuntu release. Only navigation2 (`nav2_costmap_2d`, `nav2_util`) is certainly unreleased on Rolling. The Rolling leg does not build `ais_layer`, so it needs neither.
- `tf2/utils.hpp` exists on Jazzy (geometry2 PR #723, tf2 0.36.23 on the `jazzy` branch).

## Approach

Each fix is its own commit so the backport can cherry-pick it.

1. **Baseline build** — in the worktree, source its `env.sh`, run its `build.sh` and `test.sh` for all three packages on Rolling (underlay at `/home/roland/project11/rolling/underlay`, 64 packages, built). Record the failures that appear (the four known and any others).
2. **`marine_ais_tools/setup.py`** — replace `tests_require=['pytest']` with `extras_require={'test': ['pytest']}`. Commit. Verify a Python test (e.g. `test_covariance.py`) is collected and runs on Rolling.
3. **`ais_parser.py:329`** — `self.get_logger().warn(s)` becomes `.warning(s)` (confirmed an rclpy logger). Grep the package for other deprecated calls (`.warn(`, `tf_transformations`, etc.) and fix any. Commit.
4. **`ais_layer/src/ais_layer.cpp:20`** — `tf2/utils.h` becomes `tf2/utils.hpp`. Confirmed present on Jazzy (tf2 0.36.23, geometry2 #723) and on Rolling. Commit.
5. **`ais_layer/CMakeLists.txt` geodesy** — remove the `ament_target_dependencies` block (geodesy was its only remaining entry). Add `target_include_directories(${PROJECT_NAME} PUBLIC ${geodesy_INCLUDE_DIRS})` and add `${geodesy_LIBRARIES}` to `target_link_libraries(${PROJECT_NAME} ...)`. No include-only trial: the library is needed because `ECEFPoint(GeoPoint)` is defined in `geoconv`, and an include-only form may link a SHARED plugin yet fail later at the gtest link or plugin load. The form is distro-neutral: the export shape is the same on 1.0.6 and 1.0.7. Verify on Rolling by building `ais_layer`, running its gtests, and checking the plugin library's undefined symbols resolve (`ldd -r` or the gtest load). Commit.
6. **`ais_layer/package.xml` geodesy minimum** — change `<depend>geodesy</depend>` to `<depend version_gte="1.0.7">geodesy</depend>`. Chosen so the ECEF requirement is visible at the source and rosdep/colcon surface it, instead of living only in the CI header comment. Check that `colcon build` and `rosdep` accept the attribute against the local underlay (1.0.7). Own commit, adjacent to step 5 (it is a Jazzy-relevant statement: the released Jazzy `geodesy` 1.0.6 does not satisfy it, which is correct).
7. **Further fixes** — whatever step 1 and the full rebuild turn up (e.g. `nav2_util` test target names, nav2 header moves, lint failures from newer ament linters), one commit each, in Jazzy-compatible forms.
8. **CI: `.github/workflows/ci.yml`** (own commit, standalone so it cherry-picks to `jazzy`). Modeled on marine_control's `ci.yml` (container `ros:<distro>-ros-core`, rosdep, colcon build, test, test-result). Two jobs, no matrix, so each can carry its own steps and its own `if`:
   - Triggers: `on: push` and `pull_request`, both `branches: [rolling, jazzy]`.
   - Job `jazzy` (container `ros:jazzy-ros-core`), no `if` (runs on every trigger, so it is the whole workflow on `jazzy`). Steps: checkout the repo into `src/marine_ais`; **clone `https://github.com/ros-geographic-info/geographic_info.git` at the pinned tag `1.0.7` (or a later pinned ref, verified at implementation to be a tag/branch that contains `geodesy/include/geodesy/ecef.h`) into `src/geographic_info`** before rosdep; `rosdep install --from-paths src --ignore-src -y` (`--ignore-src` skips `geodesy`, `geographic_msgs`, now present as source); `colcon build` and `colcon test` on all three packages; `colcon test-result --verbose`. Rosdep still supplies `nav2_costmap_2d` and `nav2_util` as released Jazzy packages.
   - Job `rolling` (container `ros:rolling-ros-core`) with `if: (github.base_ref || github.ref_name) != 'jazzy'`. `github.base_ref` is set on pull requests, `github.ref_name` on pushes; both are named so a PR to `jazzy` and a push to `jazzy` each skip it. Steps: checkout; `rosdep install --from-paths src/marine_ais/marine_ais_msgs src/marine_ais/marine_ais_tools --ignore-src -y` (installing from only the two selected package paths avoids the Nav2 keys, and `geodesy`, which may have no candidate on the container's Ubuntu); `colcon build --packages-up-to marine_ais_msgs marine_ais_tools` and the same for `colcon test`.
   - Header comment states: the triggers and the `jazzy` guard; what each job builds; why the Jazzy job clones geographic_info (released Jazzy `geodesy` is 1.0.6 and lacks `geodesy/ecef.h`, which `ais_layer` needs; first released as 1.0.7) and when the clone can be dropped (a Jazzy release of geographic_info 1.0.7 or later); that the Rolling job excludes `ais_layer` because navigation2 has no Rolling release, and that `ais_layer` on Rolling is not covered by CI and is checked by hand locally; that option A (build Nav2 from source in the Rolling job) is the follow-up if it proves cheap. Do not claim `geodesy` has no Rolling release: it has (1.0.7-4); it simply is not needed by the Rolling job.
   - Cherry-pick behavior: on `jazzy` the file runs only the `jazzy` job. The `rolling` job's `if` is false for any `jazzy` base or ref, so nothing changes in the cherry-picked file. The pinned clone stays in the Jazzy job on both branches.
   - The workflow cannot be run here; validate with `actionlint` or `yamllint` if installed, and by reading it against marine_control's working file. The first real run is the PR itself.
9. **Final verification** — clean build and test of all three packages on Rolling (with the underlay), including `ais_layer` and its gtests, via the worktree's `build.sh` and `test.sh`. This is the hand check of the CI gap; paste the result summary in the PR.
10. **PR to `rolling`** (host pushes) — body `Part of #14`; states what was and was not checked: Rolling all three packages locally; Jazzy only by CI (including the geographic_info 1.0.7 source clone and why); `ais_layer` on Rolling by hand only, not in CI; navigation2 is the package without a Rolling release; the backport to `jazzy` follows outside this PR (cherry-pick, PR to `jazzy`, CI as its check). Cost write-up goes to the issue as a comment, like marine_control#13.

## Files to Change

| File | Change |
|------|--------|
| `marine_ais_tools/setup.py` | `tests_require` to `extras_require={'test': ['pytest']}` |
| `marine_ais_tools/marine_ais_tools/ais_parser.py` | `.warn(` to `.warning(`, plus any other deprecated calls found |
| `ais_layer/src/ais_layer.cpp` | `tf2/utils.h` to `tf2/utils.hpp` |
| `ais_layer/CMakeLists.txt` | drop `ament_target_dependencies`; `${geodesy_INCLUDE_DIRS}` and `${geodesy_LIBRARIES}` |
| `ais_layer/package.xml` | `<depend version_gte="1.0.7">geodesy</depend>` |
| `.github/workflows/ci.yml` | new: two jobs; `jazzy` (all three packages, geographic_info 1.0.7 cloned) and `rolling` (msgs+tools, skipped on `jazzy`); branches `rolling` and `jazzy` |
| `.agent/work-plans/issue-14/` | plan and progress records, carried in the PR |
| others as the build finds | one commit each |

## Prior Art

rolker/marine_control PR #14 and its `ci.yml` (working Jazzy + Rolling matrix): taken for the workflow shape and lessons (`${<msgs>_TARGETS}` rather than aggregate targets, `extras_require`, `warning`, one commit per fix). Changed from it: two jobs instead of a matrix, because a job `if` cannot read `matrix` and so cannot select one leg. Rejected: building every package on Rolling, since `ais_layer` needs Nav2. Option A (Nav2 source build in CI) was left as a follow-up by the owner's decision. The project's own manifest (`project11.manifest.yaml`) already builds geographic_info from source; the CI clone follows the same practice.

## Serves

None (project has no written goals); no project design document. Part of rolker/unh_marine_autonomy#415.

## Design Picture

No change to the design picture; the project has no design document.

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Test what breaks | Rolling job fails without the setup.py/warn fixes; the `ais_layer` fixes are guarded by the Jazzy job and the local Rolling build, with the Rolling CI hole stated. |
| Verify before claiming | Real Rolling build of all three packages. Jazzy claims limited to what CI shows; `tf2/utils.hpp` on Jazzy and the geodesy export shape are confirmed from source, but the Jazzy build itself is only shown by CI. |
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
| Add `.github/workflows/` | triggers name `rolling` and `jazzy`; `rolling` job guarded off on `jazzy` | Yes (step 8) |
| `ais_layer` needs geodesy >= 1.0.7 | package.xml `version_gte`; Jazzy CI clones geographic_info at a pinned ref; header comment | Yes (steps 6, 8) |
| Repo has no README | nothing to update for supported distros | Yes (none exists; not adding one) |
| Code fixes | backport to `jazzy` | Outside this plan (host, by hand) |
| `ais_layer` not in Rolling CI | header comment and PR text state the gap | Yes |
| Jazzy gets a released geographic_info >= 1.0.7 | drop the clone from the `jazzy` job | Noted in header comment; follow-up |

## Open Questions

- Pinned ref for the Jazzy clone: tag `1.0.7` is the plan; confirm at implementation that the tag exists upstream and contains `ecef.h` (else use the commit that added ECEF, 491eff4, or the `ros2` branch tip).
- Does the Jazzy `ros:jazzy-ros-core` job build `geodesy` 1.0.7 cleanly from source (its dependencies `geographic_msgs`, `tf2`, etc. via rosdep)? Only CI can say.
- Does `version_gte` on `geodesy` cause any rosdep or colcon complaint on a source-built package? Checked locally at step 6; if it causes trouble, fall back to a comment in package.xml and the CI header only.

## Estimated Scope

Single PR to `rolling` (about 6 to 8 small commits); the `jazzy` backport is a second PR outside this plan.
