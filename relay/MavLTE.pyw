# Double-click to open MavLTE (runs without a console window).
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mavlte  # noqa: E402

mavlte.main()
