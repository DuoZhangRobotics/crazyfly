import pytest
from types import SimpleNamespace

from crazyfly.mock_stack import MockMotion, MockRobot, MockStack


def test_mock_motion_interpolates_instead_of_teleporting(monkeypatch) -> None:
    robot = MockRobot(position=[0.0, 0.0, 0.0])
    robot.active_motion = MockMotion(
        start=(0.0, 0.0, 0.0),
        target=(1.0, 2.0, 3.0),
        started_at=10.0,
        duration_s=2.0,
        flying_after=True,
    )
    stack = object.__new__(MockStack)
    stack.robots = {"cf1": robot}
    monkeypatch.setattr("crazyfly.mock_stack.time.monotonic", lambda: 11.0)

    stack._update_motions()

    assert robot.position == [0.5, 1.0, 1.5]
    assert robot.active_motion is not None


def test_mock_motion_applies_final_flying_state(monkeypatch) -> None:
    robot = MockRobot(position=[0.0, 0.0, 0.3], flying=True)
    robot.active_motion = MockMotion(
        start=(0.0, 0.0, 0.3),
        target=(0.0, 0.0, 0.04),
        started_at=10.0,
        duration_s=1.0,
        flying_after=False,
    )
    stack = object.__new__(MockStack)
    stack.robots = {"cf1": robot}
    monkeypatch.setattr("crazyfly.mock_stack.time.monotonic", lambda: 11.0)

    stack._update_motions()

    assert robot.position == pytest.approx([0.0, 0.0, 0.04])
    assert not robot.flying
    assert robot.active_motion is None


def test_mock_trajectory_uses_scaled_clock(monkeypatch) -> None:
    duration = SimpleNamespace(sec=2, nanosec=0)
    piece = SimpleNamespace(
        duration=duration,
        poly_x=[0.0, 1.0],
        poly_y=[0.0],
        poly_z=[0.3],
    )
    robot = MockRobot(
        position=[0.0, 0.0, 0.3],
        trajectories={1: [piece]},
        active_trajectory_id=1,
        trajectory_started_at=10.0,
        trajectory_timescale=2.0,
    )
    stack = object.__new__(MockStack)
    stack.robots = {"cf1": robot}
    monkeypatch.setattr("crazyfly.mock_stack.time.monotonic", lambda: 11.0)

    stack._update_trajectories()

    assert robot.position == pytest.approx([0.5, 0.0, 0.3])
    assert robot.active_trajectory_id == 1
