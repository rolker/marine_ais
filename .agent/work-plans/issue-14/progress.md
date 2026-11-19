---
issue: 14
---

# Issue #14 — Build on ROS 2 Rolling: port ais_layer and marine_ais_tools, add a rolling branch

## Issue Review
**Status**: complete
**When**: 2026-11-19 12:07 -05:00
**By**: Claude Code Agent (claude-sonnet-5-5)

**Issue**: #14

### Scope Assessment

**Well-scoped?** Yes. Four known edits across three packages, plus a CI decision; two PRs (rolling, then jazzy) as stated. One check is still open: `geodesy` is the only dependency left on `ament_target_dependencies` after the others moved to targets, and I could not confirm from the source that Rolling's `geodesy` exports a CMake target. The plan should test that first, since it decides the shape of the CMake fix.
**Right repo?** Yes. All changes are in this repository.
**Serves**: none (project has no written goals); no project design document.
**Design picture?** No change.
**Dependencies**: none on other issues. The underlay needs `nav2_costmap_2d`, `nav2_util` and `geodesy` on Rolling; `nav2_costmap_2d` has no Rolling release, which is the CI question below. Part of rolker/unh_marine_autonomy#415; rolker/marine_control#13 is the precedent (its write-up is a comment there).

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Test what breaks | Watch | A Rolling CI job is the check that fails without the fixes, but only if it builds and tests all three packages. If the Rolling leg cannot build `ais_layer`, it checks two of three; the plan must say so. |
| Verify before claiming | Watch | "Builds on Rolling" needs a real Rolling build, including `nav2_costmap_2d` from source on the dev machine. The issue's table is from reading source at `jazzy`, not from a build; expect more fixes, as with marine_control. |
| Small steps; step back when they stop converging | OK | Small fixes; the Jazzy-compatible forms mean the backport should be a cherry-pick. |
| Leave in a project only what it chose to carry | OK | Settled by the owner: records under `.agent/work-plans/issue-14/` ride the feature branch and PR to `rolling`. |
| Know whether it works | OK | The cost write-up on the issue is the review of the trial. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0002 Worktree isolation | Yes | Work is in the issue worktree. |
| 0019 repo_pin | Yes | PR base is `rolling` (exists at c821363, jazzy tip), then `jazzy` for the backport. |
| Others | No | |

### Consequences

- Adding CI adds `.github/workflows/` to a repo with none; its triggers should name `rolling` (and `jazzy` if the backport PR is to be checked).
- If the README states supported distros, it should say Rolling.
- A CI workflow that is added on `rolling` must not be assumed to exist on `jazzy`; the hand backport of code fixes then has no CI unless the workflow is also taken to `jazzy`.

### Recommendations

- CI options for the plan to choose between. (A) Jazzy + Rolling matrix like marine_control, where the Rolling leg builds `nav2_costmap_2d` (and `nav2_util`/`nav2_core` as needed) from source in a `ros:rolling` container via a `vcs import` of a pinned Nav2 ref: full coverage, but a long, brittle first build; mitigate by caching the Nav2 install layer or a prebuilt image. (B) Jazzy leg builds and tests all three packages; Rolling leg builds and tests only `marine_ais_msgs` and `marine_ais_tools` (`--packages-up-to` / `--packages-skip ais_layer`), and the plan states that `ais_layer` on Rolling is checked by hand locally only: cheap, with a stated hole. (C) Jazzy-only CI, Rolling checked locally: smallest, but nothing guards the Rolling fixes after merge. My lean is B first, with A as a follow-up if the Nav2 source build proves cheap; either way the issue's "what was and was not checked" line must name the Rolling `ais_layer` coverage.
- Add the workflow on `rolling` and take it to `jazzy` in the backport PR, so the backport has CI as its check (the owner has said it is done by cherry-pick with CI as its check).
- Confirm the `geodesy` form on both distros (exported target versus keeping `ament_target_dependencies`, which is gone on Rolling) and that `tf2/utils.hpp` exists on Jazzy, before claiming the fixes build on both.
- Confirm the `.warn(` at `ais_parser.py:329` is on an rclpy logger (`self.get_logger()`; it is) and grep for other deprecated calls, so "all three packages build and pass" is verified rather than assumed.

### Actions
- [ ] CI options for the plan to choose between. (A) Jazzy + Rolling matrix like marine_control, where the Rolling leg builds `nav2_costmap_2d` (and `nav2_util`/`nav2_core` as needed) from source in a `ros:rolling` container via a `vcs import` of a pinned Nav2 ref: full coverage, but a long, brittle first build; mitigate by caching the Nav2 install layer or a prebuilt image. (B) Jazzy leg builds and tests all three packages; Rolling leg builds and tests only `marine_ais_msgs` and `marine_ais_tools` (`--packages-up-to` / `--packages-skip ais_layer`), and the plan states that `ais_layer` on Rolling is checked by hand locally only: cheap, with a stated hole. (C) Jazzy-only CI, Rolling checked locally: smallest, but nothing guards the Rolling fixes after merge. My lean is B first, with A as a follow-up if the Nav2 source build proves cheap; either way the issue's "what was and was not checked" line must name the Rolling `ais_layer` coverage.
- [ ] Add the workflow on `rolling` and take it to `jazzy` in the backport PR, so the backport has CI as its check (the owner has said it is done by cherry-pick with CI as its check).
- [ ] Confirm the `geodesy` form on both distros (exported target versus keeping `ament_target_dependencies`, which is gone on Rolling) and that `tf2/utils.hpp` exists on Jazzy, before claiming the fixes build on both.
- [ ] Confirm the `.warn(` at `ais_parser.py:329` is on an rclpy logger (`self.get_logger()`; it is) and grep for other deprecated calls, so "all three packages build and pass" is verified rather than assumed.

## Checkpoint
**Status**: complete
**When**: 2026-11-19 12:10 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: owner
**After**: issue-actions
**Decision**: proceed

Two questions were asked with the Issue Review's four Actions quoted. The owner's answers, verbatim:

- Q1 "Go on to planning with F1 to F4 handed to the planner, and your Q2 answer as the CI decision?": "Proceed (Recommended)".
- Q2 "What CI should this issue add to `marine_ais`?": "B: Jazzy full, Rolling partial (Recommended)". The workflow's Jazzy leg builds and tests all three packages; its Rolling leg builds and tests `marine_ais_msgs` and `marine_ais_tools` only. `ais_layer` on Rolling is checked by hand on the development machine, and the gap is stated in the workflow and in the pull request.

Also settled by the owner before this run: the records under `.agent/work-plans/issue-14/` are carried in the pull request to `rolling`; the backport to `jazzy` is done by hand outside the loop, by cherry-pick, with CI as its check.

## Plan Authored
**Status**: complete
**When**: 2026-11-19 12:11 -05:00
**By**: Claude Code Agent (claude-sonnet-5-5)
**Plan**: `.agent/work-plans/issue-14/plan.md` at `014add3`

One commit per Rolling fix (setup.py extras_require, logger warning, tf2/utils.hpp, geodesy via include/library variables since it exports no target), then a Jazzy+Rolling CI workflow (Rolling leg msgs+tools only) triggered on rolling and jazzy so it cherry-picks to jazzy; ais_layer on Rolling checked by hand.

## Plan Review
**Status**: complete
**When**: 2026-11-19 12:13 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Verdict**: needs-work

**Issue**: #14 — Build on ROS 2 Rolling: port ais_layer and marine_ais_tools, add a rolling branch
**Plan**: `.agent/work-plans/issue-14/plan.md` at `014add3`
**Branch**: `feature/issue-14`

### Evaluation

| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | One PR to `rolling`, one commit per fix, CI commit standalone; backport outside. |
| Issue alignment | Needs work | Serves: "None (project has no written goals); no project design document. Part of rolker/unh_marine_autonomy#415." All four Issue Review actions are taken up, but the Jazzy-leg dependency claim is wrong (finding 1), so "Jazzy leg all three packages" would fail as planned. |
| File targeting | Good | Right files; `ais_layer/package.xml` may need a `geodesy` version note (finding 1). |
| Consequences | Needs work | The Jazzy leg's `geodesy` source requirement and its effect on the cherry-picked workflow on `jazzy` are not captured. |
| Principle alignment | Good | Verify-before-claiming is well kept (Jazzy claims limited to CI); one step tests a form known not to work (finding 2). |
| ADR compliance | Good | 0002, 0013, 0019 addressed. |
| ROS conventions | Needs work | CMake target/variable use is right in direction; geoconv library must be linked (finding 2). |

### Findings

1. **[Issue alignment / Consequences] (high)** — The Jazzy leg will not build `ais_layer` with rosdep's released `geodesy`. `ais_layer.cpp` includes `geodesy/ecef.h` and uses `geodesy::ECEFPoint`; ECEF support was added in geographic_info 491eff4 ("Add ECEF support…", #60), first released as 1.0.7 (tag 6ad4ef3, May 2026). rosdistro `jazzy/distribution.yaml` has geographic_info at `1.0.6-2`, whose `geodesy/include` holds only `utm.h` and `wgs84.h` (checked at 24806ad, the 1.0.6 bump, and at `origin/jazzy`). Local builds never saw this because `project11.manifest.yaml` builds geographic_info from source (`ros2`) for every distro. The plan's step 7 ("rosdep pulls … `geodesy` as released packages") therefore fails on the PR to `rolling` and again on the backport PR to `jazzy`. Resolution within option B: the Jazzy leg clones geographic_info at a pinned ref (tag `1.0.7` or later) into the CI workspace's `src/` before rosdep (`--ignore-src` then skips it), the header comment says why and when it can be dropped (a Jazzy release of geodesy ≥ 1.0.7), and optionally `ais_layer/package.xml` states `<depend version_gte="1.0.7">geodesy</depend>` so the requirement is visible.
2. **[ROS conventions] (medium)** — The geodesy library is needed; drop the "include dirs alone" experiment in step 5. `ECEFPoint(const geographic_msgs::msg::GeoPoint &)` is declared in `ecef.h` but defined in `src/conv/ecef_conversions.cpp`, which is compiled into the exported static library `geoconv`. Because `ais_layer` is a SHARED library, an include-only form may still link and only fail at the gtest link or at plugin load, which reads like a success at step 5. Use `target_include_directories(${PROJECT_NAME} PUBLIC ${geodesy_INCLUDE_DIRS})` plus `${geodesy_LIBRARIES}` in `target_link_libraries`. On the open question: the export shape is the same on 1.0.6 (`origin/jazzy`) and 1.0.7/`ros2` (`ament_export_libraries(geoconv)`, `ament_export_include_directories(include)`, no exported target), so this form is distro-neutral once finding 1 supplies 1.0.7 on Jazzy.
3. **[ROS conventions / CI design] (medium)** — The guard as written ("job `if` … so a PR or push to `jazzy` runs the Jazzy leg only") does not work on a single matrix job: `jobs.<id>.if` cannot read the `matrix` context, so it can only skip both legs. Use either two jobs (a Jazzy job, and a Rolling job with `if: (github.base_ref || github.ref_name) != 'jazzy'`), or a matrix computed by expression from `github.base_ref || github.ref_name` (with `include` entries carrying the per-leg `packages` field). `github.base_ref` covers pull requests, `github.ref_name` covers pushes; the plan names both, keep both. Two jobs also make the Jazzy-only source checkout of finding 1 simpler.
4. **[Consequences] (low)** — Factual correction for the header comment and PR text: Rolling does have a `geodesy` release (rosdistro rolling geographic_info `1.0.7-4`); only navigation2 lacks a Rolling release. The Rolling leg's rosdep step therefore needs to avoid only the Nav2 keys; installing from the selected package paths (`--from-paths` naming `marine_ais_msgs` and `marine_ais_tools`) is the simplest form.
5. **[Issue alignment] (info)** — `tf2/utils.hpp` exists on Jazzy: added to the `jazzy` branch of ros2/geometry2 by "Jazzy 'Backport' of #720 (#723)" on 2024-12-19 and present at the branch tip (tf2 0.36.23). Step 4 is safe; the plan's "unverified" can be dropped.

### Summary

The fix list and commit structure are sound and the review-issue actions are carried, but the CI Jazzy leg as planned would fail on `ais_layer` because Jazzy's released `geodesy` (1.0.6) predates the ECEF headers the code uses, and the proposed per-leg guard cannot be expressed as a job `if` on a matrix job. Fix those two points (and link `geoconv`) before implementing.

### Recommended Actions

- [ ] Step 7: Jazzy leg builds geographic_info from source at a pinned ref (≥ 1.0.7) before rosdep; state why in the workflow header; consider `version_gte="1.0.7"` on the `geodesy` depend.
- [ ] Step 5: link `${geodesy_LIBRARIES}` and use `${geodesy_INCLUDE_DIRS}`; drop the include-only trial.
- [ ] Step 7: implement the `jazzy` guard as two jobs or an expression-computed matrix, not a job `if` on the matrix job.
- [ ] Correct the "geodesy has no Rolling release" wording; restrict the Rolling leg's rosdep to the selected package paths.
- [ ] Step 4: mark `tf2/utils.hpp` on Jazzy as confirmed (geometry2 #723).

## Checkpoint
**Status**: complete
**When**: 2026-11-19 12:14 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: owner
**After**: plan
**Decision**: revise

Asked "How should the plan go forward?" with the Plan Review's verdict (needs-work) and its findings 1 to 5 quoted. The owner answered: "Revise (Recommended)".

Host note for the planner, on Plan Review finding 4: rosdistro lists geographic_info 1.0.7-4 for Rolling, but on the development machine (Ubuntu 26.04, resolute) `apt-cache policy ros-rolling-geodesy` shows no candidate while `ros-rolling-geographic-msgs` has one, and `rosdep resolve geodesy` names `ros-rolling-geodesy`. So whether the Rolling CI container can install `geodesy` depends on its Ubuntu release; the Rolling leg does not build `ais_layer`, so it need not. The project's underlay layer (navigation2, grid_map, geographic_info from source) is now built at /home/roland/project11/rolling/underlay.

## Plan Authored
**Status**: complete
**When**: 2026-11-19 12:15 -05:00
**By**: Claude Code Agent (claude-sonnet-5-5)
**Plan**: `.agent/work-plans/issue-14/plan.md` at `32dfa8d`

Revision after the Plan Review: Jazzy job clones geographic_info 1.0.7 before rosdep and package.xml states geodesy version_gte 1.0.7; geodesy is linked via include dirs plus geoconv library; CI is two jobs with the Rolling job guarded by an if on base_ref or ref_name; the Rolling-release facts are corrected; tf2/utils.hpp on Jazzy is confirmed.

## Plan Review
**Status**: complete
**When**: 2026-11-19 12:17 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Verdict**: ready

**Issue**: #14 — Build on ROS 2 Rolling: port ais_layer and marine_ais_tools, add a rolling branch
**Plan**: `.agent/work-plans/issue-14/plan.md` at `32dfa8d`
**Branch**: `feature/issue-14`

### Earlier findings (Plan Review of `014add3`)

| # | Finding | Status in `32dfa8d` |
|---|---|---|
| 1 | Jazzy leg cannot build `ais_layer` with released `geodesy` 1.0.6 | Resolved. Jazzy job clones geographic_info at tag `1.0.7` before rosdep; header comment says why and when to drop it; `version_gte` added. Checked: tag `1.0.7` exists upstream (`git ls-remote` gives 6ad4ef3, on `origin/ros2`), contains `geodesy/include/geodesy/ecef.h`, and has 491eff4 (ECEF support) as an ancestor. |
| 2 | Link `geoconv`; drop include-only trial | Resolved. Step 5 uses `${geodesy_INCLUDE_DIRS}` and `${geodesy_LIBRARIES}`, no trial, with a symbol check. (See new finding 3 on PUBLIC vs PRIVATE.) |
| 3 | Job `if` cannot read `matrix` | Resolved. Two jobs; `rolling` job has `if: (github.base_ref \|\| github.ref_name) != 'jazzy'`. Checked: on `pull_request` `base_ref` is the target branch; on `push` it is empty, so `ref_name` is used. PR or push to `jazzy` skips the job; PR or push to `rolling` runs it. Cherry-picked unchanged onto `jazzy` it runs only the `jazzy` job. |
| 4 | Rolling does have a `geodesy` release; rosdep from selected paths | Resolved. Wording corrected; Rolling job's rosdep names only the two package paths. Checked locally: `ros-rolling-tf-transformations`, `ros-rolling-nmea-msgs`, `ros-rolling-geographic-msgs` all have resolute candidates, so the Rolling job's keys should resolve. |
| 5 | `tf2/utils.hpp` on Jazzy confirmed | Resolved. Step 4 and Context state it as confirmed. |

### Evaluation

| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | One PR to `rolling`, about 6 to 8 commits, CI commit standalone; backport outside the loop as the owner fixed. |
| Issue alignment | Good | Serves: "None (project has no written goals); no project design document. Part of rolker/unh_marine_autonomy#415." All four Issue Review actions and option B carried; Rolling `ais_layer` gap stated in workflow header and PR text. |
| File targeting | Good | Right files; `ais_layer/package.xml` added for the version floor. |
| Consequences | Good | geodesy >= 1.0.7 consequence (package.xml, Jazzy clone, header, drop-clone follow-up) captured. |
| Principle alignment | Good | Jazzy claims limited to CI; Rolling `ais_layer` hand check named. Step 6's stated benefit is overstated (finding 1). |
| ADR compliance | Good | 0002, 0013, 0019 addressed. |
| ROS conventions | Good | geodesy include/library form is right and distro-neutral (1.0.7 `geodesy/CMakeLists.txt`: `add_library(geoconv STATIC ...)`, `ament_export_libraries(geoconv)`, no exported target). Minor visibility point (finding 3). |

### Findings

1. **[Principle alignment / ROS conventions] (low)** — Step 6 says `version_gte="1.0.7"` makes "rosdep/colcon surface it". Neither does: rosdep resolves keys by name only (no version handling in `rosdep2`), and colcon-ros records `version_gte` as dependency metadata without enforcing it. So the attribute is safe on Jazzy (rosdep resolves `geodesy` to the released package where no source is present, and `--ignore-src` skips it in CI), but its effect is documentation plus bloom's Debian `Depends: ros-jazzy-geodesy (>= 1.0.7)` if the package is ever released. Step 6's "check that colcon and rosdep accept the attribute" will pass trivially and the third Open Question can be closed; correct the rationale so it does not claim a check that does not exist (the real signal on a 1.0.6 machine is the missing `geodesy/ecef.h` compile error).
2. **[Consequences / CI design] (low)** — Cloning geographic_info at `1.0.7` puts three more packages in `src/` (`geographic_info`, `geographic_msgs`, `geodesy`). Make the Jazzy job's colcon scope explicit so this repo's check does not run geodesy's own tests and linters: e.g. `colcon build --packages-up-to ais_layer marine_ais_tools` and `colcon test --packages-select marine_ais_msgs marine_ais_tools ais_layer` (and `colcon test-result` over those). The source `geographic_msgs` 1.0.7 overlays the released 1.0.6; harmless, since between tags `1.0.6` and `1.0.7` it changes only `CHANGELOG.rst` and `package.xml`. A shallow clone (`--depth 1 --branch 1.0.7`) keeps it cheap.
3. **[ROS conventions] (low)** — Step 5 adds `${geodesy_INCLUDE_DIRS}` as `PUBLIC`. `geodesy` is used only in `src/ais_layer.cpp`; no header under `ais_layer/include` includes it. `PUBLIC` exports that absolute include path in `ais_layer`'s exported target for no consumer's benefit; use `PRIVATE`. Keep `${geodesy_LIBRARIES}` in the existing plain-signature `target_link_libraries` call (adding a keyword there would mix signatures on one target, which CMake rejects).

### Summary

The revision resolves all five earlier findings, and the pinned tag, the geodesy export shape and the `jazzy` guard check out against the sources. The three new points are low-severity wording and scoping corrections that can be taken during implementation. Ready.

### Recommended Actions

- [ ] Step 6: reword the `version_gte` rationale (documentation and bloom Depends; not enforced by rosdep or colcon); close the third Open Question.
- [ ] Step 8: scope the Jazzy job's colcon build/test to this repo's packages; consider a shallow clone.
- [ ] Step 5: make `${geodesy_INCLUDE_DIRS}` `PRIVATE`.

## Checkpoint
**Status**: complete
**When**: 2026-11-19 12:39 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: owner
**After**: plan
**Decision**: proceed

Asked "Implement this plan, with F1 to F3 handed to the implementer as corrections?" with the revised plan (32dfa8d) summarised and the second Plan Review's three low findings quoted (F1: the `version_gte` rationale claims a check that does not exist; F2: make the Jazzy job's colcon scope explicit and clone geographic_info shallow; F3: `${geodesy_INCLUDE_DIRS}` is PRIVATE, `${geodesy_LIBRARIES}` stays in the plain-signature `target_link_libraries` call). The owner answered: "Proceed (Recommended)". The three findings are corrections to the plan for the implementer.

## Implementation
**Status**: complete
**When**: 2026-11-19 12:43 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Branch**: feature/issue-14 at `88038f4`

Baseline on Rolling (before changes): `ais_layer` failed to configure ("Unknown CMake command \"ament_target_dependencies\""); `marine_ais_tools` warned "Unknown distribution option: 'tests_require'" and `colcon test` collected nothing ("NO TESTS RAN", exit 5); `marine_ais_msgs` xmllint failed ("Element 'test_depend': This element is not expected. Expected is one of ( member_of_group, export )").

Code commits (one fix each, Jazzy-compatible forms):
- `d7a90a0` marine_ais_tools/setup.py: `tests_require` to `extras_require={'test': ['pytest']}`.
- `4e3187e` ais_parser.py: logger `.warn` to `.warning` (no other deprecated calls found).
- `0053112` marine_ais_msgs/package.xml: `test_depend` before `member_of_group` (found by the baseline; not in the plan's list).
- `b4dd9e1` ais_layer.cpp: `tf2/utils.h` to `tf2/utils.hpp`.
- `623fdde` ais_layer/CMakeLists.txt: `ament_target_dependencies` removed; `${geodesy_INCLUDE_DIRS}` PRIVATE, `${geodesy_LIBRARIES}` in the existing plain `target_link_libraries` call.
- `270ab39` ais_layer/package.xml: `<depend version_gte="1.0.7">geodesy</depend>` (documentation and a Debian Depends if released; not enforced by rosdep or colcon).
- `a64e56e` ais_layer.cpp: local helper replaces `Layer::declareParameter`, removed from Nav2 on Rolling (navigation2 #5867); does what Jazzy's version did (`has_parameter`/`declare_parameter` on `getFullName`). Found by the build.
- `0e3a7cc` ais_layer.cpp: `rclcpp::create_subscription(node, ...)` instead of the node member, which Rolling's `nav2::LifecycleNode` replaces with a (topic, callback, qos) signature. Found by the build.

CI commit: `88038f4` `.github/workflows/ci.yml`, two jobs: `jazzy` (shallow clone of geographic_info tag 1.0.7, build `--packages-up-to` the three packages, test `--packages-select` the three) and `rolling` (msgs and tools only, `if: (github.base_ref || github.ref_name) != 'jazzy'`); header states the `ais_layer` Rolling gap.

Final clean build and test on Rolling (underlay at /home/roland/project11/rolling/underlay): build "Summary: 3 packages finished"; `test_ais_layer` "[  PASSED  ] 35 tests.", `test_plugin_loading` "[  PASSED  ] 1 test.", ais_layer linters "100% tests passed, 0 tests failed out of 7", marine_ais_tools "27 passed", marine_ais_msgs "100% tests passed, 0 tests failed out of 3"; `colcon test-result --verbose`: "Summary: 90 tests, 0 errors, 0 failures, 4 skipped" (the 4 skipped are ament_cppcheck's own skips in ais_layer). `ldd -r libais_layer.so`: 0 undefined symbols; `geodesy::ECEFPoint::ECEFPoint(GeoPoint)` is defined in the library (geoconv linked statically).

Not checked: Jazzy (not installed, no Docker), so the Jazzy forms of the code fixes and the workflow are shown only by CI on the PR. The workflow was not run; YAML parsed and read against marine_control's working file (no actionlint/yamllint installed). The tests do not call `onInitialize`, so the two Nav2 API fixes are shown by compilation only, not exercised at runtime. Left as is: `marine_ais_msgs` CMake deprecation warning (`cmake_minimum_required(VERSION 3.8)`, below 3.10) and an unused `tf_transformations` depend in marine_ais_tools/package.xml.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-11-19 12:51 -05:00
**By**: Claude Code Agent (claude-opus-5-5)
**Verdict**: approved

**Branch**: feature/issue-14 at `f068f9d`
**Base**: origin/rolling
**Depth**: Standard (reason: adds `.github/workflows/ci.yml`)
**Must-fix**: 0 | **Suggestions**: 5
**Round**: 1 | **Ship**: recommended — no must-fix findings; remaining suggestions can be applied or tracked

Checked for the two unplanned commits: a64e56e's helper matches Jazzy's `Layer::declareParameter` (navigation2 `origin/jazzy`: `declare_parameter_if_not_declared(node, getFullName(p), value)`; the only other effect, `local_params_`, is never read). 0e3a7cc's free `rclcpp::create_subscription(node, ...)` is what Jazzy's `rclcpp_lifecycle::LifecycleNode::create_subscription` calls, with default options and the node's default callback group, as before; it bypasses Rolling Nav2's wrapper, so no `qos_overrides.*` parameters, which matches Jazzy. A scratch program on Rolling (outside the repo, deleted) loaded the plugin, called `initialize` twice on a `nav2::LifecycleNode` with overrides and a pre-declared `ais.enabled=false`: overrides read, pre-declared value kept, no re-declare throw, subscription on the overridden topic with depth 50, reliable, volatile, no QoS override parameters.

### Findings
- [x] (suggestion) Jazzy job: the clone also builds `geographic_msgs` (and geodesy's unused gtests); `nav2_msgs` on Jazzy depends on `geographic_msgs`, so the binary is installed anyway and the header's "--ignore-src then skips ... geographic_msgs" is wrong. Build only geodesy from the clone (`COLCON_IGNORE` in `geographic_msgs` and `geographic_info`) and correct the comment — `.github/workflows/ci.yml:9-10,64-69`
- [x] (suggestion) Drop `-r` from both `rosdep install` steps: with it rosdep returns 0 when a key cannot be resolved (rosdep2/main.py), so a missing dependency shows up later as a build or import error — `.github/workflows/ci.yml:75,140`
- [x] (suggestion) `find_package(geodesy 1.0.7 REQUIRED)`: ament's version file accepts >= 1.0.7, so a 1.0.6 geodesy fails at configure with a clear message instead of a missing `geodesy/ecef.h` — `ais_layer/CMakeLists.txt:13`
- [x] (suggestion) Plain `target_link_libraries` puts the absolute path of `libgeoconv.a` in the exported target's link interface; use the keyword form with `${geodesy_LIBRARIES}` PRIVATE (the archive is already inside the plugin) — `ais_layer/CMakeLists.txt:38-39`
- [x] (suggestion) Add a gtest that calls `initialize()` (LayeredCostmap, tf2_ros::Buffer, lifecycle node with parameter overrides; node type via `#if __has_include("nav2_ros_common/lifecycle_node.hpp")`: `nav2::LifecycleNode` vs `nav2_util::LifecycleNode`) and asserts parameters and the subscription, so the Jazzy CI job exercises a64e56e and 0e3a7cc, not only compiles them — `ais_layer/test/test_ais_layer.cpp`

## Checkpoint
**Status**: complete
**When**: 2026-11-19 12:51 -05:00
**By**: Claude Code Agent (claude-fable-5-2)
**Decided-by**: owner
**After**: publish
**Decision**: address

Asked "Publish now, or take the suggestions first?" with the pre-push review's verdict (round 1 approved, 0 must-fix, 5 suggestions) and the five suggestions quoted. The owner answered: "Address all five first (Recommended)". All five suggestions of the Local Review (Pre-Push) above are to be addressed before the branch is pushed.

## Implementation
**Status**: complete
**When**: 2026-11-19 13:00 -05:00
**By**: Claude Code Agent (claude-opus-5-5)

**Branch**: feature/issue-14 at `4c4bd16`
**Addressed**: Local Review (Pre-Push) at `f068f9d` (2026-11-19 12:51 -05:00)
**Commits**: `60177d0`, `1c99696` (CI); `ffb596c`, `0668537`, `3a6fdca` (code); `4c4bd16` (box checks)

### Actions
- [x] Jazzy job builds only geodesy from the clone: `COLCON_IGNORE` in `geographic_msgs` and `geographic_info`, geodesy built first with `-DBUILD_TESTING=OFF`, then the three packages with `--packages-skip geodesy`; header comment corrected (rosdep installs the released geographic_msgs; 1.0.6..1.0.7 of geographic_msgs differs only in version and changelog, checked with `git diff --stat 1.0.6 1.0.7`) — `.github/workflows/ci.yml` (`60177d0`). Checked locally: catkin_pkg's `find_package_paths` honours `COLCON_IGNORE`; `colcon list` on a scratch workspace with the 1.0.7 clone lists only geodesy, ais_layer, marine_ais_msgs, marine_ais_tools.
- [x] `-r` dropped from both `rosdep install` steps — `.github/workflows/ci.yml` (`1c99696`). Every key resolved: Jazzy job (all `package.xml` under `src`, geodesy 1.0.7's included, `rosdep resolve --rosdistro jazzy --os ubuntu:noble`), 28 keys, all to `ros-jazzy-*` or `python3-catkin-pkg`/`python3-pyproj`/`python3-pytest`/`python3-serial`, `tf_transformations -> ros-jazzy-tf-transformations`; `rosdep install --simulate ... --rosdistro jazzy --os ubuntu:noble` exit 0. Rolling job (msgs + tools), 14 keys, all resolve for `ubuntu:resolute` (`tf_transformations -> ros-rolling-tf-transformations`); none of the ROS keys resolve for `ubuntu:noble` on Rolling.
- [x] `find_package(geodesy 1.0.7 REQUIRED)` — `ais_layer/CMakeLists.txt:13` (`ffb596c`). Underlay geodesy's `geodesyConfig-version.cmake` says 1.0.7; a scratch CMake project found it for 1.0.7 and did not for 1.0.8.
- [x] Keyword `target_link_libraries`: PUBLIC geographic_msgs, geometry_msgs, marine_ais_msgs, nav2_costmap_2d_core, rclcpp (included by `include/ais_layer/ais_layer.hpp`); PRIVATE `${geodesy_LIBRARIES}`, pluginlib, tf2, tf2_geometry_msgs, tf2_ros (src only; the header names tf2 only in a comment) — `ais_layer/CMakeLists.txt` (`0668537`). Installed `export_ais_layerExport.cmake`: no `geoconv` (grep count 0); `ldd -r libais_layer.so`: 0 undefined/not found; `geodesy::ECEFPoint::ECEFPoint(GeoPoint const&)` defined (`T`) in the plugin; gtests link.
- [x] New `ais_layer/test/test_initialize.cpp` (`3a6fdca`): five tests on a lifecycle node with overrides (overrides read; `ais.enabled` declared false before `initialize()` stays false; second `initialize()` does not throw; subscription `/ais_contacts` with `count_subscribers == 1`; default topic `/contacts`). A mutation run (helper declaring unconditionally) failed the pre-declared and second-initialize tests, then was reverted. No new `find_package`/`test_depend`: it links ais_layer, nav2_costmap_2d_core, nav2_util_core, rclcpp, tf2_ros, all already present; `nav2_ros_common` reaches the test on Rolling through nav2_costmap_2d_core and is not named, because it has no Jazzy package and rosdep without `-r` would fail on it.

### Jazzy signatures checked (navigation2 `origin/jazzy` at `c7046550`, by `git show`)
- `Layer::initialize(LayeredCostmap *, std::string, tf2_ros::Buffer *, const nav2_util::LifecycleNode::WeakPtr &, rclcpp::CallbackGroup::SharedPtr)` (layer.hpp:73, layer.cpp body identical to main's: sets members, locks the node for logger and clock, calls `onInitialize()`).
- `nav2_util::LifecycleNode` defines no `WeakPtr` of its own, so `WeakPtr` is `std::weak_ptr<rclcpp_lifecycle::LifecycleNode>`; the test passes `std::shared_ptr<nav2_util::LifecycleNode>`, an implicit derived-to-base conversion.
- `nav2_util::LifecycleNode(const std::string &, const std::string & ns = "", const rclcpp::NodeOptions & = {})`; the test calls `(name, "", options)`, valid on both. Its constructor declares only bond/autostart parameters.
- `LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown)`, same on both.
- Jazzy `layer.hpp` includes `tf2_ros/buffer.h` and `nav2_util/lifecycle_node.hpp`; the test includes no tf2_ros header itself; `nav2_ros_common/lifecycle_node.hpp` does not exist on Jazzy, so `__has_include` picks `nav2_util`.

### Checks (Rolling, clean build: `core_ws/build`, `install`, `log` removed)
- build.sh: "Summary: 3 packages finished [17.6s]".
- test.sh exit 0: `test_ais_layer` "[  PASSED  ] 35 tests.", `test_initialize` "[  PASSED  ] 5 tests.", `test_plugin_loading` "[  PASSED  ] 1 test.", ais_layer "100% tests passed, 0 tests failed out of 8", marine_ais_tools "27 passed", marine_ais_msgs "100% tests passed, 0 tests failed out of 3".
- `colcon test-result --verbose`: "Summary: 99 tests, 0 errors, 0 failures, 5 skipped" (all 5 are cppcheck's per-file skips).
- `ais_layer.cpp` compiled `-fsyntax-only` against the geodesy 1.0.7 tag's headers (the CI clone): rc 0. The tag's `ecef.h` lacks `#include <iomanip>` (added after the tag in f70b81a); it compiles here through transitive includes.

### Not verified (no Jazzy, no Docker, workflow not run)
- The Jazzy build of `test_initialize.cpp` and of the CMake changes; the signatures above were read, not compiled.
- That the `ros:rolling-ros-core` image is Ubuntu resolute (Rolling's keys resolve only there); that `--packages-skip geodesy` in the second colcon call picks up the geodesy installed by the first in the CI container (colcon's normal behaviour); that the 1.0.7 tag's `ecef.h` compiles in ais_layer under Noble's GCC 13 without `<iomanip>`.
- `count_subscribers` relies on the RMW updating the local graph synchronously (true for the default Fast DDS here).

Full pre-commit pass left to the host.
