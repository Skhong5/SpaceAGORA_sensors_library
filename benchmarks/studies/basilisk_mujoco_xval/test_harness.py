"""Bounded harness checks; no Julia, Basilisk, MuJoCo, SSH or benchmark runs.

Run with a Python environment containing NumPy and tomllib/tomli:
    python3 -m unittest discover -s benchmarks/studies/basilisk_mujoco_xval -p test_harness.py
"""
import csv
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


HERE = Path(__file__).resolve().parent


class RemoteStatusTests(unittest.TestCase):
    def run_wrapper(self, status=0, warning=False):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            shutil.copyfile(HERE / "run_case.sh", root / "run_case.sh")
            (root / "bin").mkdir()
            # Retained outputs must never make a failed remote invocation look successful.
            stale = root / "results/sf3/caseA_T30/bsk_dt0.025.csv"
            stale.parent.mkdir(parents=True)
            stale.write_text("retained results\n")
            programs = {
                "ssh": f"echo remote-output; echo remote-stderr >&2; "
                       f"{'echo WARNING; ' if warning else ''}exit {status}\n",
                "rsync": 'echo rsync >> "$STEP_LOG"\n',
                "flock": 'echo julia >> "$STEP_LOG"\n',
                "python3": 'echo compare >> "$STEP_LOG"; echo comparison-passed\n',
            }
            for name, body in programs.items():
                program = root / "bin" / name
                program.write_text("#!/bin/sh\n" + body)
                program.chmod(0o755)
            env = dict(os.environ, PATH=str(root / "bin") + os.pathsep + os.environ["PATH"],
                       TMPDIR=td, STEP_LOG=str(root / "steps"))
            result = subprocess.run(["sh", str(root / "run_case.sh"), "caseA", "30", "unused"],
                                    capture_output=True, text=True, env=env)
            steps = (root / "steps").read_text().splitlines() if (root / "steps").exists() else []
            self.assertEqual(stale.read_text(), "retained results\n")
            self.assertEqual(list(root.glob("tmp.*")), [], "temporary log was not removed")
            self.assertIn("remote-output", result.stdout)
            self.assertIn("remote-stderr", result.stdout)
            return result, steps

    def test_failed_remote_aborts_before_retained_results_or_local_work(self):
        result, steps = self.run_wrapper(status=137)
        self.assertEqual(result.returncode, 137)
        self.assertEqual(steps, [])
        self.assertIn("failed with status 137", result.stdout)

    def test_success_preserves_output_and_runs_remaining_steps(self):
        result, steps = self.run_wrapper()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(steps, ["rsync", "julia", "compare"])
        self.assertIn("comparison-passed", result.stdout)

    def test_successful_remote_warning_still_aborts(self):
        result, steps = self.run_wrapper(warning=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(steps, [])


class ComparisonTimeTests(unittest.TestCase):
    def run_comparison(self, sagora_times=(0, 1), basilisk_times=(0, 1), bad_level=None):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            (root / "case.toml").write_text('name = "synthetic"\n[[body]]\nname = "hub"\n')
            for tool, times in [("sagora", sagora_times), ("bsk", basilisk_times)]:
                folder = root / tool
                folder.mkdir()
                for h in [0.04, 0.02, 0.01]:
                    level_times = bad_level if tool == "sagora" and h == 0.02 and bad_level else times
                    with (folder / f"{tool}_dt{h}.csv").open("w", newline="") as f:
                        writer = csv.writer(f)
                        writer.writerow(["t", "hub_qw", "hub_qx", "hub_qy", "hub_qz",
                                         "hub_wx", "hub_wy", "hub_wz"])
                        writer.writerows([t, 1, 0, 0, 0, h, 0, 0] for t in level_times)
            result = subprocess.run([sys.executable, str(HERE / "compare.py"), str(root / "case.toml"),
                                     str(root / "sagora"), str(root / "bsk"), str(root / "out")],
                                    capture_output=True, text=True)
            rows = []
            if (root / "out.csv").exists():
                with (root / "out.csv").open() as f:
                    rows = list(csv.DictReader(f))
            return result, rows

    def test_aligned_ladders_preserve_numerical_acceptance(self):
        result, rows = self.run_comparison()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(len(rows), 2)
        for row in rows:
            self.assertEqual(float(row["bound"]), 10 * max(float(row["e_S"]), float(row["e_B"])))
            self.assertEqual(row["passed"], "True")
            self.assertEqual(float(row["D"]), 0)

    def test_shifted_cross_tool_times_are_rejected(self):
        result, rows = self.run_comparison(basilisk_times=(10, 11))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("SpaceAGORA and Basilisk sample times differ", result.stderr)
        self.assertEqual(rows, [])

    def test_different_cross_tool_sample_counts_are_rejected(self):
        result, rows = self.run_comparison(basilisk_times=(0, 1, 2))
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(rows, [])

    def test_existing_sampling_tolerance_is_preserved(self):
        result, _ = self.run_comparison(basilisk_times=(5e-10, 1 + 5e-10))
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_within_ladder_shift_is_rejected(self):
        result, rows = self.run_comparison(bad_level=(10, 11))
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(rows, [])

    def test_nonfinite_times_are_rejected(self):
        for value in [float("nan"), float("inf"), -float("inf")]:
            with self.subTest(value=value):
                result, rows = self.run_comparison(basilisk_times=(0, value))
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(rows, [])


if __name__ == "__main__":
    unittest.main()
