"""sitl_demo.py pieces that run without SITL."""

import argparse
import os
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import sitl_demo  # noqa: E402


class StartSitlTest(unittest.TestCase):
    def test_relative_sitl_path(self):
        # SITL runs in its own work folder, so the program path must not stay relative (on Linux the
        # child looks it up from there)
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        exe = os.path.join(tmp.name, "ardupilot", "arduplane")
        os.makedirs(os.path.dirname(exe))
        open(exe, "w").close()
        cwd = os.getcwd()
        os.chdir(tmp.name)
        self.addCleanup(os.chdir, cwd)
        args = argparse.Namespace(sitl=os.path.join("ardupilot", "arduplane"), speedup=1.0, home=None, wipe=False)
        with mock.patch.object(sitl_demo.tempfile, "gettempdir", return_value=tmp.name), \
                mock.patch.object(sitl_demo.subprocess, "Popen") as popen:
            sitl_demo.start_sitl(args)
        popen.call_args.kwargs["stdout"].close()  # the log file, which a real Popen would hand to SITL
        program = popen.call_args[0][0][0]
        self.assertTrue(os.path.isabs(program), program)
        self.assertTrue(os.path.samefile(program, exe))


if __name__ == "__main__":
    unittest.main()
