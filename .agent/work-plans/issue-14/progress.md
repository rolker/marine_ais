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
