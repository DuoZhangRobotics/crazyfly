from dataclasses import replace
from math import dist
from pathlib import Path

import pytest
import yaml

from crazyfly.config import ConfigError, load_fleet, load_safety
from crazyfly.config import point_in_geofence_frame
from crazyfly.safety import continuous_minimum_separation
from crazyfly.trajectory import (
    PolynomialPiece,
    RobotTrajectory,
    TrajectoryPlan,
    derivative_coefficients,
    evaluate_coefficients,
    evaluate_piece,
    load_trajectory,
    trajectory_from_payload,
    trajectory_in_world,
    trajectory_payload,
    validate_compiled_trajectory,
)


ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "config" / "trajectories" / "mock_square.yaml"


def _inputs():
    return (
        load_fleet(ROOT / "config" / "mock_crazyflies.yaml"),
        load_safety(ROOT / "config" / "mock_safety.yaml"),
    )


def _write_modified(tmp_path: Path, edit) -> Path:
    source = yaml.safe_load(EXAMPLE.read_text())
    edit(source)
    path = tmp_path / "trajectory.yaml"
    path.write_text(yaml.safe_dump(source, sort_keys=False))
    return path


def _coefficients(*values: float) -> tuple[float, ...]:
    return tuple(values) + (0.0,) * (8 - len(values))


def _piece(
    duration_s: float,
    x: tuple[float, ...],
    y: tuple[float, ...] = (0.0,),
    z: tuple[float, ...] = (0.3,),
) -> PolynomialPiece:
    return PolynomialPiece(
        duration_s=duration_s,
        poly_x=_coefficients(*x),
        poly_y=_coefficients(*y),
        poly_z=_coefficients(*z),
        poly_yaw=(0.0,) * 8,
    )


def test_version_two_square_compiles_and_is_continuous() -> None:
    plan = load_trajectory(EXAMPLE, *_inputs())

    assert plan.name == "mock_square"
    assert plan.frame == "base"
    assert plan.duration_s == 16.0
    assert plan.metrics is not None
    trajectory = plan.trajectories["cf1"]
    assert len(trajectory.pieces) == 4
    for index, piece in enumerate(trajectory.pieces):
        assert evaluate_piece(piece, 0.0) == pytest.approx(
            plan.waypoints[index].goals["cf1"].position
        )
        assert evaluate_piece(piece, piece.duration_s) == pytest.approx(
            plan.waypoints[index + 1].goals["cf1"].position
        )
    for order in range(1, 4):
        assert evaluate_piece(trajectory.pieces[0], 0.0, order) == pytest.approx(
            (0.0, 0.0, 0.0), abs=1e-8
        )
        assert evaluate_piece(
            trajectory.pieces[-1], trajectory.pieces[-1].duration_s, order
        ) == pytest.approx((0.0, 0.0, 0.0), abs=1e-8)
        for first, second in zip(trajectory.pieces, trajectory.pieces[1:]):
            assert evaluate_piece(first, first.duration_s, order) == pytest.approx(
                evaluate_piece(second, 0.0, order), abs=1e-8
            )
    internal_velocity = evaluate_piece(
        trajectory.pieces[0], trajectory.pieces[0].duration_s, 1
    )
    assert dist(internal_velocity, (0.0, 0.0, 0.0)) > 1e-3


def test_parser_rejects_version_one(tmp_path: Path) -> None:
    path = _write_modified(tmp_path, lambda data: data.update(version=1))
    with pytest.raises(ConfigError, match="version 2"):
        load_trajectory(path, *_inputs())


def test_parser_requires_zero_start_and_strict_shared_times(tmp_path: Path) -> None:
    path = _write_modified(
        tmp_path, lambda data: data["waypoints"][0].update(time_s=1.0)
    )
    with pytest.raises(ConfigError, match="must be zero"):
        load_trajectory(path, *_inputs())

    path = _write_modified(
        tmp_path, lambda data: data["waypoints"][2].update(time_s=4.0)
    )
    with pytest.raises(ConfigError, match="strictly increasing"):
        load_trajectory(path, *_inputs())


def test_parser_requires_every_robot_and_zero_yaw(tmp_path: Path) -> None:
    path = _write_modified(
        tmp_path, lambda data: data["waypoints"][0].update(goals={})
    )
    with pytest.raises(ConfigError, match="exactly the enabled robots"):
        load_trajectory(path, *_inputs())

    path = _write_modified(
        tmp_path,
        lambda data: data["waypoints"][1]["goals"]["cf1"].update(yaw_deg=5.0),
    )
    with pytest.raises(ConfigError, match="require yaw_deg 0"):
        load_trajectory(path, *_inputs())


def test_compiler_enforces_continuous_derivative_limits(tmp_path: Path) -> None:
    def compress(data):
        for index, waypoint in enumerate(data["waypoints"]):
            waypoint["time_s"] = index * 0.1

    path = _write_modified(tmp_path, compress)
    with pytest.raises(ConfigError, match="trajectory (speed|acceleration|jerk)"):
        load_trajectory(path, *_inputs())


def test_continuous_geofence_catches_between_waypoint_excursion() -> None:
    _, safety = _inputs()
    plan = TrajectoryPlan(
        name="hump",
        frame="base",
        duration_s=4.0,
        trajectories={
            "cf1": RobotTrajectory(
                pieces=(_piece(4.0, (0.0, 2.0, -0.5)),)
            )
        },
    )

    with pytest.raises(ConfigError, match="soft geofence"):
        validate_compiled_trajectory(plan, safety)


def test_continuous_separation_catches_crossing_between_endpoints() -> None:
    _, safety = _inputs()
    plan = TrajectoryPlan(
        name="crossing",
        frame="base",
        duration_s=4.0,
        trajectories={
            "cf1": RobotTrajectory(pieces=(_piece(4.0, (-0.5, 0.25)),)),
            "cf2": RobotTrajectory(pieces=(_piece(4.0, (0.5, -0.25)),)),
        },
    )

    with pytest.raises(ConfigError, match="separation"):
        validate_compiled_trajectory(plan, safety)


def test_payload_round_trip_is_revalidated() -> None:
    fleet, safety = _inputs()
    plan = load_trajectory(EXAMPLE, fleet, safety)
    payload = trajectory_payload(plan, mission_id="mission-1", trajectory_id=7)

    mission_id, trajectory_id, restored = trajectory_from_payload(
        payload, tuple(fleet.enabled), safety
    )

    assert mission_id == "mission-1"
    assert trajectory_id == 7
    assert restored.start_positions == pytest.approx(plan.start_positions)
    assert restored.end_positions == pytest.approx(plan.end_positions)
    assert restored.metrics == plan.metrics


def test_payload_rejects_corrupt_polynomial_coefficients() -> None:
    fleet, safety = _inputs()
    plan = load_trajectory(EXAMPLE, fleet, safety)
    payload = trajectory_payload(plan, mission_id="mission-1", trajectory_id=7)
    payload["trajectories"]["cf1"]["pieces"][0]["poly_x"] = [0.0]

    with pytest.raises(ConfigError, match="eight finite coefficients"):
        trajectory_from_payload(payload, tuple(fleet.enabled), safety)


def test_base_to_world_polynomial_transform_uses_inverse_direction() -> None:
    _, safety = _inputs()
    base_from_world = (
        (0.0, -1.0, 0.0, 1.0),
        (1.0, 0.0, 0.0, 2.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )
    safety = replace(safety, geofence_from_world=base_from_world)
    piece = PolynomialPiece(
        duration_s=1.0,
        poly_x=_coefficients(1.0, 1.0),
        poly_y=_coefficients(3.0, 0.0),
        poly_z=_coefficients(0.3, 0.0),
        poly_yaw=(0.0,) * 8,
    )
    plan = TrajectoryPlan(
        name="transform",
        frame="base",
        duration_s=1.0,
        trajectories={"cf1": RobotTrajectory(pieces=(piece,))},
    )

    world = trajectory_in_world(plan, safety)["cf1"].pieces[0]

    assert (world.poly_x[0], world.poly_y[0], world.poly_z[0]) == pytest.approx(
        (1.0, 0.0, 0.3)
    )
    assert (world.poly_x[1], world.poly_y[1], world.poly_z[1]) == pytest.approx(
        (0.0, -1.0, 0.0)
    )


def test_derivative_helpers_use_ascending_polynomial_coefficients() -> None:
    derivative = derivative_coefficients((1.0, 2.0, 3.0), 1)
    assert derivative == pytest.approx((2.0, 6.0))
    assert evaluate_coefficients(derivative, 2.0) == pytest.approx(14.0)


def test_four_drone_example_prepositions_to_line_with_planning_clearance() -> None:
    fleet = load_fleet(ROOT / "config" / "local" / "crazyflies.yaml")
    safety = load_safety(ROOT / "config" / "local" / "safety.yaml")
    plan = load_trajectory(
        ROOT / "config" / "trajectories" / "four_drone_box.yaml",
        fleet,
        safety,
    )
    launch_hover = {
        name: (
            point_in_geofence_frame(robot.initial_position, safety)[0],
            point_in_geofence_frame(robot.initial_position, safety)[1],
            0.30,
        )
        for name, robot in fleet.enabled.items()
    }

    minimum, _pair, _time_fraction = continuous_minimum_separation(
        launch_hover, plan.start_positions
    )

    assert minimum >= safety.trajectory_minimum_separation_m
    assert plan.metrics is not None
    assert plan.metrics.minimum_separation_m == pytest.approx(0.35, abs=1e-6)
