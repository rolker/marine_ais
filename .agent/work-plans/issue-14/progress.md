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
