from pathlib import Path
import math
import sys


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from SCARA_UI.motion.motion_mixin import ScaraMotionMixin
from SCARA_UI.trajectory.look_ahead import LookAheadPlanner


class GeometryOwner(ScaraMotionMixin):
    def __init__(self):
        self.cur_x = 75.0
        self.cur_y = 220.0
        self.path_planner = LookAheadPlanner(accel_mm_s2=100.0, junction_deviation=0.02)
        self.junction_dev = 0.02
        self.errors = []

    def ui_to_mcu_xy(self, x, y):
        return x - 75.0, y

    def _read_run_accel_mm_s2(self):
        return 80.0

    def validate_trajectory_points(self, _points, label="trajectory"):
        return True

    def log_error(self, message):
        self.errors.append(str(message))


def check_real_geometry_gcode():
    owner = GeometryOwner()
    line = owner.path_planner.line_segment((75.0, 220.0), (85.0, 220.0))
    arc = owner.path_planner.arc_segment((85.0, 220.0), (95.0, 230.0), 10.0, clockwise=False)
    commands = owner.generate_geometry_gcode([line, arc], 20.0, start=(75.0, 220.0))
    assert commands[0].startswith("G1 ")
    assert commands[1].startswith("G3 ")
    assert " I" in commands[1] and " J" in commands[1]
    assert len(commands) == 2


def check_look_ahead():
    planner = LookAheadPlanner(accel_mm_s2=100.0, junction_deviation=0.02)
    straight = [
        planner.line_segment((0.0, 0.0), (30.0, 0.0)),
        planner.line_segment((30.0, 0.0), (60.0, 0.0)),
    ]
    corner = [
        planner.line_segment((0.0, 0.0), (30.0, 0.0)),
        planner.line_segment((30.0, 0.0), (30.0, 30.0)),
    ]
    planner.plan_segments(straight, 20.0)
    planner.plan_segments(corner, 20.0)
    assert math.isclose(straight[1].entry_speed, 20.0, rel_tol=0.05)
    assert corner[1].entry_speed < straight[1].entry_speed

    sharp = [
        planner.line_segment((0.0, 0.0), (30.0, 0.0)),
        planner.line_segment((30.0, 0.0), (4.019, 15.0)),
    ]
    planner.plan_segments(sharp, 20.0)
    assert sharp[1].entry_speed < corner[1].entry_speed

    reversal = [
        planner.line_segment((0.0, 0.0), (30.0, 0.0)),
        planner.line_segment((30.0, 0.0), (0.0, 0.0)),
    ]
    planner.plan_segments(reversal, 20.0)
    assert reversal[1].entry_speed == 0.0


def check_stroke_geometry_stream():
    owner = GeometryOwner()
    strokes = [
        [(75.0, 220.0), (85.0, 220.0), (85.0, 230.0)],
        [(95.0, 230.0), (105.0, 230.0)],
    ]
    preview, command_source, command_count = owner.generate_stroke_motion(strokes, 20.0, label="test writing")
    commands = list(command_source)
    geometry = [line for line in commands if line.startswith(("G1 ", "G2 ", "G3 "))]
    assert preview
    assert geometry
    assert len(geometry) < len(preview)
    assert all("F1200" in line for line in geometry)
    assert commands.count("G4 P0.001") >= 2
    assert any(line.startswith("G0 ") for line in commands)
    assert command_count == len(commands)
    assert math.isclose(owner.path_planner.accel_mm_s2, 80.0)


def main():
    check_real_geometry_gcode()
    check_look_ahead()
    check_stroke_geometry_stream()
    print("TRAJECTORY_PLANNER_CHECK PASS")


if __name__ == "__main__":
    main()
