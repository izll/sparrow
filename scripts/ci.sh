#!/usr/bin/env bash
#
# The complete verification gate — every check that has to pass before this branch is releasable,
# in one command.
#
# It exists because the second audit found the opposite: the ten binary-level regression tests were
# all `#[ignore]`d, so the documented `cargo test --release` ran none of them, and the two Python
# self-tests were not part of any documented gate at all. A gate that has to be assembled by hand
# from four half-remembered commands is a gate that gets run in half-remembered form. This script
# is that assembly, written down once.
#
#   scripts/ci.sh
#
# Every stage is fatal (`set -e`); the first failure stops the run and its exit code becomes the
# script's. Nothing here is optional or "best effort" — a check that may be skipped silently is the
# exact failure mode being fixed.

set -euo pipefail

cd "$(dirname "$0")/.."

# Bold section headers, but only when stdout is a terminal (CI logs get plain text).
if [ -t 1 ]; then B=$'\033[1m'; N=$'\033[0m'; else B=''; N=''; fi
step() { echo; echo "${B}=== $* ===${N}"; }

step "1/6  cargo test  (debug assertions ON — invariant violations)"
# The test profile keeps `debug_assert!`, so the walled/BPP invariants guarded by one are live here.
cargo test --locked

step "2/6  cargo test --release  (debug assertions OFF — what users actually run)"
# Where the criticals lived: with the assertions compiled out, a bad value flows on into the
# exported JSON instead of tripping an assert. Both audits' criticals were of exactly this shape.
# The binary-level E2E suites are NOT ignored any more, so this stage runs them.
cargo test --release --locked

step "3/6  cargo test --release -- --ignored  (the wall-clock suites)"
# The handful of tests that assert on elapsed time. They are `#[ignore]`d because a loaded machine
# makes them flaky and a debug build makes them meaningless — but "not part of the default run" must
# not mean "never run", which is why they are here.
cargo test --release --locked -- --ignored

step "4/6  validate_solution.py --self-test"
# The independent geometric validator. It is the only check that reads the exported JSON with no
# shared code with the engine, so its own correctness is load-bearing.
python3 scripts/validate_solution.py --self-test

step "5/6  nest_race.py --self-test"
# The race harness: engine selection, staleness, the CPU budget, and the result.json lifecycle.
# Runs entirely on fake engines, so it needs no binaries and no wall clock.
python3 scripts/nest_race.py --self-test

step "6/6  cargo clippy --all-targets"
cargo clippy --all-targets --locked -- -D warnings

echo
echo "${B}ALL CHECKS PASSED${N}"
