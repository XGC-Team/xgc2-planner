#!/usr/bin/env python3
"""Configure-only regression: real CMakeLists; fake deps and empty C++ inputs.

No solver is linked, no ROS node starts, and these checks are not native smoke.
Requires Python 3, PyYAML, CMake, make and C/C++ compilers. All outputs are isolated.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import unittest

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
LOG_DIR = None
SERIAL = 0


def delay_errors(config):
    """Only the two mandatory timing keys; not a safety-bound certificate."""
    errors = []
    tuning = config.get("tuning_param", {})
    for key in ("delay_check_sec", "simulated_comm_delay_sec"):
        value = tuning.get(key)
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value < 0 or (key == "delay_check_sec" and value == 0)):
            errors.append(key)
    return errors


class ConfigureFixture:
    def __init__(self, root, cmake_text):
        self.root = Path(root)
        self.source = self.root / "source"
        self.source.mkdir(parents=True)
        (self.source / "CMakeLists.txt").write_text(cmake_text, encoding="utf-8")
        # Only filenames needed for generation, never solver implementations.
        for name in set(re.findall(r"\bsrc/[\w/]+(?:\.cpp)?", cmake_text)):
            path = self.source / name
            if not path.suffix:
                path = path.with_suffix(".cpp")
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("// Empty configure-only fixture. DO NOT BUILD.\n")
        modules = self.root / "finders"
        modules.mkdir()
        for name in ("Eigen3", "CGAL", "catkin", "decomp_util", "NLOPT", "GUROBI"):
            text = "set(%s_FOUND TRUE)\n" % name
            if name == "CGAL":
                text += 'set(CGAL_USE_FILE "${CMAKE_CURRENT_LIST_DIR}/CGALUse.cmake")\n'
            elif name == "catkin":
                text += ('macro(catkin_package)\nendmacro()\n'
                         'add_custom_target(fixture_messages)\n'
                         'set(catkin_EXPORTED_TARGETS fixture_messages)\n'
                         'set(CATKIN_PACKAGE_BIN_DESTINATION bin)\n'
                         'set(CATKIN_PACKAGE_SHARE_DESTINATION share/rmader)\n')
            (modules / ("Find" + name + ".cmake")).write_text(text)
        (modules / "CGALUse.cmake").write_text("# Configure-only fixture.\n")
        self.modules = modules
        self.env = os.environ.copy()
        self.env["GUROBI_HOME"] = str(self.root / "not-a-gurobi-install")
        self.build = self.root / "build"

    def configure(self, selection=None):
        global SERIAL
        SERIAL += 1
        command = ["cmake", "-S", str(self.source), "-B", str(self.build),
                   "-G", "Unix Makefiles", "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
                   "-DCMAKE_MODULE_PATH=" + str(self.modules)]
        if selection is not None:
            command += ["-DUSE_GUROBI=" + selection]
        result = subprocess.run(command, env=self.env, text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                timeout=25, check=False)
        receipt = {"command": command, "returncode": result.returncode,
                   "scope": "configure-only; fake package discovery; empty C++ inputs",
                   "output": result.stdout}
        if LOG_DIR is not None:
            LOG_DIR.mkdir(parents=True, exist_ok=True)
            (LOG_DIR / ("configure-%02d.json" % SERIAL)).write_text(
                json.dumps(receipt, indent=2) + "\n")
        if result.returncode:
            raise AssertionError(json.dumps(receipt, indent=2))
        commands = json.loads((self.build / "compile_commands.json").read_text())
        solvers = [(Path(item["file"]).name, item["command"]) for item in commands
                   if Path(item["file"]).name in ("solver_nlopt.cpp", "solver_gurobi.cpp")]
        if len(solvers) != 1:
            raise AssertionError("Expected exactly one solver translation unit: %r" % solvers)
        filename, command_line = solvers[0]
        selected = "ON" if filename == "solver_gurobi.cpp" else "OFF"
        flag = "1" if selected == "ON" else "0"
        if "USE_GUROBI_FLAG=" + flag not in command_line:
            raise AssertionError("Compile definition and selected source disagree: " + command_line)
        print("CONFIGURE %02d requested=%s selected=%s flag=%s" %
              (SERIAL, selection or "cached/default", selected, flag), flush=True)
        return selected


class BackendContract(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="rmader-backend-")
        self.addCleanup(self.tmp.cleanup)
        self.text = (PACKAGE / "CMakeLists.txt").read_text()

    def fixture(self, name="case", text=None):
        return ConfigureFixture(Path(self.tmp.name) / name,
                                self.text if text is None else text)

    def test_default_selects_nlopt(self):
        self.assertEqual(self.fixture().configure(), "OFF")

    def test_gurobi_survives_reconfigure_without_flag(self):
        fixture = self.fixture()
        self.assertEqual(fixture.configure("ON"), "ON")
        self.assertEqual(fixture.configure(), "ON")
        self.assertIn("USE_GUROBI:BOOL=ON", (fixture.build / "CMakeCache.txt").read_text())

    def test_explicit_override_is_respected(self):
        fixture = self.fixture()
        self.assertEqual(fixture.configure("ON"), "ON")
        self.assertEqual(fixture.configure("OFF"), "OFF")
        self.assertEqual(fixture.configure(), "OFF")

    def test_isolated_builds_do_not_share_selection(self):
        gurobi, nlopt = self.fixture("gurobi"), self.fixture("nlopt")
        self.assertEqual(gurobi.configure("ON"), "ON")
        self.assertEqual(nlopt.configure("OFF"), "OFF")
        self.assertEqual(gurobi.configure(), "ON")
        self.assertEqual(nlopt.configure(), "OFF")

    def test_negative_control_reintroducing_unset_causes_drift(self):
        fixture = self.fixture(text=self.text + "\nunset(USE_GUROBI CACHE)\n")
        self.assertEqual(fixture.configure("ON"), "ON")
        self.assertEqual(fixture.configure(), "OFF")


class ParameterContract(unittest.TestCase):
    def test_default_has_both_required_timing_parameters(self):
        config = yaml.safe_load((PACKAGE / "param/rmader.yaml").read_text())
        self.assertEqual(delay_errors(config), [])

    def test_invalid_or_missing_parameters_are_detected(self):
        for key in ("delay_check_sec", "simulated_comm_delay_sec"):
            for invalid in (None, True, "0.05", -0.01, float("nan"), float("inf")):
                with self.subTest(key=key, invalid=invalid):
                    config = {"tuning_param": {"delay_check_sec": 0.05,
                                               "simulated_comm_delay_sec": 0.0}}
                    config["tuning_param"][key] = invalid
                    self.assertEqual(delay_errors(config), [key])
        self.assertEqual(delay_errors({}), ["delay_check_sec", "simulated_comm_delay_sec"])

    def test_zero_injection_is_not_zero_delay_check(self):
        self.assertEqual(delay_errors({"tuning_param": {"delay_check_sec": 0.05,
                                                        "simulated_comm_delay_sec": 0.0}}), [])
        self.assertEqual(delay_errors({"tuning_param": {"delay_check_sec": 0.0,
                                                        "simulated_comm_delay_sec": 0.0}}),
                         ["delay_check_sec"])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package-root", type=Path, default=PACKAGE)
    parser.add_argument("--log-dir", type=Path)
    args, unittest_args = parser.parse_known_args()
    PACKAGE = args.package_root.resolve()
    LOG_DIR = args.log_dir.resolve() if args.log_dir else None
    if LOG_DIR and LOG_DIR.exists() and any(LOG_DIR.iterdir()):
        parser.error("Refusing to overwrite nonempty evidence directory")
    for tool in ("cmake", "make", "cc", "c++"):
        if shutil.which(tool) is None:
            parser.error("Required configure tool missing: " + tool)
    unittest.main(argv=[sys.argv[0]] + unittest_args, verbosity=2)
