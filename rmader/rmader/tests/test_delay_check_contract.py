#!/usr/bin/env python3
"""Red contract tests for the known B02 delay-check retry defect.

Compile exact delimited C++ source bodies with a scripted checker/clock. This is
NOT a ROS/native planner test and demonstrates no actual robot collision.
--candidate tests a transient, failure-latching candidate, NOT a source fix.
"""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

SOURCE = Path(__file__).resolve().parents[1] / "src/rmader_ros.cpp"
CANDIDATE = False
LOG_DIR = None


def fragment(source, mode):
    start = "// " + mode + " delay check "
    end = "// end of " + mode + " delay check "
    begin = source.index("\n", source.index(start)) + 1
    body = source[begin:source.index(end, begin)]
    if CANDIDATE:
        # Candidate only: preserve a false result; final recheck is allowed only
        # after all prior checks succeeded. Does not address time or threading.
        if mode == "constant":
            body = body.replace("if (!delay_check_result_)", "if (delay_check_result_)")
        else:
            marker = "mtx_adaptive_dc_.unlock();"
            before, after = body.split(marker, 1)
            body = before + marker + "\nif (delay_check_result_) {\n" + after + "\n}\n"
    return body


def run_case(mode, first_failure):
    text = SOURCE.read_text(encoding="utf-8")
    body = fragment(text, mode)
    program = r'''
#include <iostream>
static double elapsed = 0;
struct MyTimer {
  explicit MyTimer(bool) { elapsed = 0; }
  double ElapsedMs() const { return elapsed * 1000; }
};
struct Mutex { void lock() {} void unlock() {} } mtx_adaptive_dc_;
namespace ros {
struct Duration {
  double value;
  explicit Duration(double v) : value(v) {}
  void sleep() { elapsed += value; }
};
}
struct Checker {
  int calls = 0;
  bool fail_first;
  bool delayCheck(int, double) { return !(calls++ == 0 && fail_first); }
};
int main() {
  Checker checker;
  checker.fail_first = FIRST_FAILURE;
  auto* rmader_ptr_ = &checker;
  const int pwp_now_ = 0;
  const double headsup_time = 0;
  const double delay_check_ = 0.05;
  const double adaptive_delay_check_ = 0.05;
  bool delay_check_result_ = false;
  {
BODY
  }
  std::cout << delay_check_result_ << " " << checker.calls << " " << elapsed << "\n";
}
'''.replace("FIRST_FAILURE", "true" if first_failure else "false").replace("BODY", body)
    with tempfile.TemporaryDirectory(prefix="rmader-dc-") as directory:
        root = Path(directory)
        (root / "probe.cpp").write_text(program)
        commands = [["c++", "-std=c++14", "-Wall", "-Wextra", str(root / "probe.cpp"),
                     "-o", str(root / "probe")], [str(root / "probe")]]
        receipts = []
        for command in commands:
            result = subprocess.run(command, text=True, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, timeout=15, check=False)
            receipts.append({"command": command, "returncode": result.returncode,
                             "output": result.stdout})
            if result.returncode:
                raise AssertionError(json.dumps(receipts, indent=2))
        output = receipts[-1]["output"].strip()
        if LOG_DIR is not None:
            LOG_DIR.mkdir(parents=True, exist_ok=True)
            key = mode + ("-fail-first" if first_failure else "-all-clear")
            (LOG_DIR / (key + ".json")).write_text(json.dumps({
                "candidate_not_applied": CANDIDATE, "source": str(SOURCE),
                "program": program, "receipts": receipts}, indent=2) + "\n")
        accepted, count, seconds = output.split()
        print("DC mode=%s first_failure=%s candidate=%s result=%s calls=%s elapsed=%s" %
              (mode, first_failure, CANDIDATE, accepted, count, seconds), flush=True)
        return accepted == "1", int(count), float(seconds)


class DelayCheckContract(unittest.TestCase):
    def test_constant_must_not_reverse_first_failure(self):
        accepted, count, _ = run_case("constant", True)
        self.assertFalse(accepted)
        self.assertEqual(count, 1)

    def test_adaptive_must_not_reverse_first_failure(self):
        accepted, count, _ = run_case("adaptive", True)
        self.assertFalse(accepted)
        self.assertEqual(count, 1)

    def test_all_clear_waits_for_window(self):
        for mode in ("constant", "adaptive"):
            with self.subTest(mode=mode):
                accepted, _, elapsed = run_case(mode, False)
                self.assertTrue(accepted)
                self.assertGreaterEqual(elapsed, 0.05)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=SOURCE)
    parser.add_argument("--candidate", action="store_true")
    parser.add_argument("--log-dir", type=Path)
    args, extra = parser.parse_known_args()
    SOURCE, CANDIDATE = args.source.resolve(), args.candidate
    LOG_DIR = args.log_dir.resolve() if args.log_dir else None
    if shutil.which("c++") is None:
        parser.error("C++ compiler required")
    if LOG_DIR and LOG_DIR.exists() and any(LOG_DIR.iterdir()):
        parser.error("Refusing to overwrite nonempty evidence directory")
    unittest.main(argv=[sys.argv[0]] + extra, verbosity=2)
