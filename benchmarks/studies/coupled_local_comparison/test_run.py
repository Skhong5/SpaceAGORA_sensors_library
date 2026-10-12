"""Bounded runner tests: mock locked points, never launch Julia or a workload."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class RunnerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.source = self.root / "source"
        harness = self.source / "benchmarks/studies/coupled_local_comparison"
        harness.mkdir(parents=True)
        self.runner = harness / "run.sh"
        shutil.copy2(Path(__file__).with_name("run.sh"), self.runner)
        self.runner.chmod(0o755)
        for tree in (self.source, self.root / "pre", self.root / "post"):
            files = tree / "benchmarks/studies/parallelization_performance"
            files.mkdir(parents=True)
            for name in ("cases.jl", "cli.jl", "execution.jl"):
                (files / name).write_text("# e6_actuated_saved mock harness\n")
        self.bin = self.root / "bin"
        self.bin.mkdir()
        # GNU realpath -m is not available on every developer host.
        realpath = self.bin / "realpath"
        realpath.write_text("#!/bin/sh\n[ \"$1\" != -m ] || shift\nprintf '%s\\n' \"$1\"\n")
        realpath.chmod(0o755)
        date = self.bin / "date"
        date.write_text("#!/bin/sh\nprintf '%s\\n' '2026-10-12T00:00:00Z'\n")
        date.chmod(0o755)
        flock = self.bin / "flock"
        flock.write_text("""#!/bin/sh
shift
mode=$6
printf '%s\n' "$mode" >> "$MOCK_CALLS"
if [ "$mode" = serial ]; then exit "$MOCK_SERIAL_RC"; fi
exit 0
""")
        flock.chmod(0o755)
        self.out = self.root / "out"
        self.calls = self.root / "calls"
        self.env = dict(os.environ, PATH=f"{self.bin}:{os.environ['PATH']}",
                        PRE_TREE=str(self.root / "pre"), POST_TREE=str(self.root / "post"),
                        LOCKS=str(self.root / "mock.lock"), SWEEP_N="2", SWEEP_THREADS="2",
                        ROUTES="rhs_flat", QUIET_WAIT_S="0", QUIET_RETRY_S="0",
                        MOCK_CALLS=str(self.calls), MOCK_SERIAL_RC="0")

    def run_stage(self, rc):
        self.env["MOCK_SERIAL_RC"] = str(rc)
        return subprocess.run(["bash", str(self.runner), "B", str(self.out)],
                              env=self.env, text=True, capture_output=True, timeout=10)

    def test_success_and_resume(self):
        first = self.run_stage(0)
        self.assertEqual(first.returncode, 0, first.stdout + first.stderr)
        self.assertIn("[done]", first.stdout)
        self.assertEqual(self.calls.read_text().splitlines(), ["serial", "rhs_flat"])
        second = self.run_stage(23)
        self.assertEqual(second.returncode, 0, second.stdout + second.stderr)
        self.assertEqual(second.stdout.count("[skip]"), 2)
        self.assertEqual(self.calls.read_text().splitlines(), ["serial", "rhs_flat"])

    def test_failure_continues_but_fails_stage(self):
        result = self.run_stage(23)
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("exit=23", result.stdout)
        self.assertNotIn("[done]", result.stdout)
        self.assertEqual(self.calls.read_text().splitlines(), ["serial", "rhs_flat"])
        self.assertFalse((self.out / "post/stack2_e6_actuated_saved_serial_t1/done").exists())
        self.assertTrue((self.out / "post/stack2_e6_actuated_saved_rhs_flat_t2/done").exists())
        resumed = self.run_stage(0)
        self.assertEqual(resumed.returncode, 0, resumed.stdout + resumed.stderr)
        self.assertEqual(resumed.stdout.count("[skip]"), 1)

    def test_quiet_timeout_continues_but_fails_stage(self):
        result = self.run_stage(75)
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("machine not quiet", result.stdout)
        self.assertNotIn("[done]", result.stdout)
        self.assertEqual(self.calls.read_text().splitlines(), ["serial", "rhs_flat"])
        self.assertFalse((self.out / "post/stack2_e6_actuated_saved_serial_t1/done").exists())
        self.assertTrue((self.out / "post/stack2_e6_actuated_saved_rhs_flat_t2/done").exists())


if __name__ == "__main__":
    unittest.main(verbosity=2)
