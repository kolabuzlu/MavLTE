# Double-click to open the plane simulator (runs without a console window).
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import plane_sim  # noqa: E402

plane_sim.main()
