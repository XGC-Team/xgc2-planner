# RMADER backend and delay-check contract — B02 / harness #150

Scope: this is a **partial delivery**, not native/scientific acceptance.
Tracking: https://github.com/XGC-Team/xgc2-harness/issues/150
Source baseline: `09deda586b8cf2e56940c769054491db30c77f6d`, target `noetic`.

## Implemented here

`USE_GUROBI` now remains in the CMake cache. A bare reconfigure must not silently
change Gurobi to NLopt; explicit `-DUSE_GUROBI=OFF` still changes it. Configuration
prints the chosen backend. Use separate build/devel/install spaces for each backend.

The default YAML restores the two required timing values from the existing comments:
`delay_check_sec=0.05` and `simulated_comm_delay_sec=0.05`. Other defaults, including
`is_artificial_comm_delay=true`, are unchanged. These are example settings, **not a
validated delay bound**. A clean parameter server previously lacked both mandatory
keys; `mu::safeGetParam` exits on a missing key. `rmader.launch` uses
`clear_params=false`, so stale server parameters can conceal the missing defaults.

No solver equations, message definitions, runtime trajectory callbacks or harness
injector are changed in this PR. The open runtime defects below are **not fixed**.

## Reproduce the checks

From a complete checkout, using Python 3 + PyYAML + CMake + make + C/C++ compilers:

```sh
python3 rmader/rmader/tests/test_backend_contract.py --log-dir /tmp/b02-config-new
python3 rmader/rmader/tests/test_delay_check_contract.py --log-dir /tmp/b02-dc-new
python3 rmader/rmader/tests/test_delay_check_contract.py --candidate --log-dir /tmp/b02-dc-candidate-new
```

The first suite executes the real package CMakeLists with fake package discovery
and empty C++ files, and inspects generated solver source/compile definitions.
It does not build/link a solver, validate a Gurobi license, or start ROS.
Before this change: 8 tests, 3 failures. After: 8 tests passed. A mutation that
restores the cache `unset` still recreates the backend drift.

The second suite deliberately remains **red** on the unchanged runtime source:
3 tests, 2 failures. It extracts the actual delay-loop bodies and compiles them
with a scripted checker/clock; `false` followed by `true` returns success in zero
simulated elapsed time in both modes. This is a control-flow witness, **not** a
native collision trace. The all-clear positive case waits for the configured window.
`--candidate` applies an in-memory failure-latching candidate to those fragments:
3 tests pass, but **no running source is modified** and this is not a native fix.
Keep default failing tests visible; do not convert them to silent skips/xfails.

## Backend identity and jerk

Gurobi `SolverGurobi::addConstraints` explicitly bounds each segment's physical
jerk through `par_.j_max`. The NLopt constructor copies velocity/acceleration bounds,
not `par.j_max`; its `j_max_` also names an unrelated normal-variable index. A jerk
cost or zero terminal jerk is not a pointwise jerk bound. Therefore an NLopt result
must not be labelled an equivalent Gurobi reproduction merely because YAML values
match. A03 must choose and identify the backend and comparison budget explicitly.
No jerk constraint is added to NLopt here.

An exact analytic witness is archived in paper-dmpc: four C2-connected cubics with
jerks `[120,-120,-120,120]`, each lasting 0.1 s, start/end at rest, satisfy
max |v| = 1.2 and max |a| = 12, but violate |j| <= 30. This disproves only the generic
implication from velocity/acceleration limits to jerk limits, not feasibility of
these coefficients in the full NLopt problem.

## Paper-to-code mapping and OPEN runtime defects

Reference: Kondo et al., arXiv:2303.06222v6, Section II, Algorithms 1–2,
Proposition 1: https://arxiv.org/abs/2303.06222v6 . The paper's delay guarantee is
conditional on maximum actual delivery delay fitting inside the delay-check window;
complete communication loss is outside it. Its DC failure rule discards the candidate.

Code-side first findings were sealed before this writer read the paper. The later
cross-check is **self-review**, not independent or blind review. Diagram/table
screenshots were unavailable; the complete case proof is not independently accepted.

At the baseline, `replanCB` publishes the candidate with `is_committed=false`, runs
DC, then installs a successful plan and publishes `true`; only then is `pwp_last_`
replaced. Failure paths retain the old plan and conditionally rebroadcast it at low
speed. Retaining a plan is not, by itself, evidence that it is still safe.

1. **Failure reversal.** Constant DC breaks on false, then rechecks under
   `if (!delay_check_result_)`; adaptive DC unconditionally rechecks after the loop.
   A later true can erase an early failure. The candidate tested here gates final
   recheck on the accumulated true result. A native regression and independent
   review are required before changing the callback. The constant loop also needs
   an end-of-window receipt/check-order audit.
2. **Injection changes storage semantics.** In `trajCB`, no artificial delay calls
   `updateTrajObstacles` (one trajectory per agent), while the artificial-delay timer
   calls `updateTrajObstacles_with_delaycheck` (append candidates, replace that
   agent's set only on commitment). Thus simply disabling artificial delay for P02
   does NOT preserve RMADER's two-stage storage. Both paths must dispatch according
   to `is_delaycheck_`, not according to whether a timer was used. This routing
   candidate is documented, not implemented or validated here.
3. **Clock/order contract.** DC timing uses `chrono::high_resolution_clock` while
   waiting uses `ros::Duration`. Clock monotonicity, paused/scaled simulation time,
   callback queuing, out-of-order committed messages and final checks remain open.
   `time_received` is captured before the internal timer, not at store application.
   `time_created` is set near publication; commented `time_sent` code is not a real
   event log. The reported missed-message count counts received messages that are
   late; it is not a count of messages that never arrived.

## P02 / E01 / A03 handoff — integration blocked

P02 owns the only external injection point. Request `run_id`, sender/receiver IDs,
message and proposal/commit correlation IDs, creation, actual publish, callback
receive, store-apply and plan-install events with explicit clock domains. Log the
realized delay/drop/reorder schedule and seed. Once the storage-route defect is
fixed, external injection must disable the internal artificial delay and record its
configured delay as zero. Neither two injected delays nor one receipt timestamp
can stand in for this contract.

A03 must freeze backend/license/dependency versions, geometry, dynamics and budgets.
E01 must provide true collision/clearance and raw Record→Analysis identities. Do not
mix scene A geometry with scene B communication results or report absent metrics as
zero. Native positive, over-bound and missing-message cases are all **NOT RUN**.

The isolated environment has no ROS/docker and no full local checkout. An unmocked
CMake dependency probe failed at Eigen3 (exit 1). Solver builds/licenses, ROS launch,
8-agent scenes, native timing/failure reuse, and an independent reviewer remain
open. No author station, robot, APT publication or default-branch merge was used.

Paper evidence: `lxk36/paper-dmpc/research/tro-26-0979-v1/B02/attempt-01/`.
Merge order: review/merge this planner PR first; the paper evidence PR pins its
commit SHA. Harness wiring requires a separate PR after the runtime/interface gates.
