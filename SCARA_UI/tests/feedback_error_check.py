import importlib.util
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


feedback_mod = load_module("feedback_error", "SCARA_UI/core/feedback_error.py")


def assert_close(got, want, tol, label):
    if abs(got - want) > tol:
        raise AssertionError(f"{label}: got {got:.6f}, want {want:.6f}")


def main():
    tracker = feedback_mod.FeedbackErrorTracker()
    tracker.set_expected_path([(0.0, 0.0), (10.0, 0.0), (20.0, 0.0)])

    sample1 = tracker.add_feedback(5.0, 0.30)
    sample2 = tracker.add_feedback(12.0, -0.40)
    sample3 = tracker.add_feedback(20.5, 0.0)
    stats = tracker.stats()

    assert_close(sample1.err_x, 0.0, 1e-6, "sample1 err_x")
    assert_close(sample1.err_y, 0.30, 1e-6, "sample1 err_y")
    assert_close(sample2.err_x, 0.0, 1e-6, "sample2 err_x")
    assert_close(sample2.err_y, -0.40, 1e-6, "sample2 err_y")
    assert_close(sample3.err_x, 0.50, 1e-6, "sample3 err_x")
    assert_close(sample3.err_y, 0.0, 1e-6, "sample3 err_y")

    assert stats.count == 3, f"count mismatch: {stats.count}"
    assert_close(stats.max_abs_x, 0.50, 1e-6, "max_abs_x")
    assert_close(stats.max_abs_y, 0.40, 1e-6, "max_abs_y")
    assert_close(stats.max_norm, 0.50, 1e-6, "max_norm")
    assert_close(stats.rms_norm, math.sqrt((0.09 + 0.16 + 0.25) / 3.0), 1e-6, "rms_norm")

    tracker.set_expected_path([])
    assert tracker.add_feedback(1.0, 1.0) is None, "empty expected path should not produce samples"
    print("FEEDBACK_ERROR_CHECK PASS")


if __name__ == "__main__":
    main()
