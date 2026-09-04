from dataclasses import replace
from types import SimpleNamespace
from math import pi
from pathlib import Path

import pytest

from crazyfly import prrtc_demo
from crazyfly.prrtc_bundle import ExecutionBundle, JointSample, JointTrajectory


def _bundle():
    main = JointTrajectory(
        "main",
        (JointSample(0.0, (0.0,) * 6), JointSample(4.0, (0.4,) * 6)),
    )
    park = JointTrajectory(
        "park",
        (JointSample(0.0, (0.4,) * 6), JointSample(2.0, (0.2,) * 6)),
    )
    return ExecutionBundle(
        root=Path("."),
        bundle_id="bundle-1",
        drone_payload={"duration_s": 4.0},
        main=main,
        park=park,
        validation={},
        manifest={},
        robot_names=("cf1",),
    )


def test_combined_demo_is_hardware_free_by_default(monkeypatch, capsys) -> None:
    monkeypatch.setattr(prrtc_demo, "load_execution_bundle", lambda *_a, **_k: _bundle())
    monkeypatch.setattr(
        prrtc_demo.WorldBaseTransform, "load", lambda *_a, **_k: object()
    )
    monkeypatch.setattr(
        prrtc_demo,
        "run_combined",
        lambda *_a, **_k: (_ for _ in ()).throw(
            AssertionError("dry run executed hardware")
        ),
    )

    result = prrtc_demo.main(["bundle"])

    assert result == 0
    assert "DRY RUN" in capsys.readouterr().out


def test_combined_execution_requires_matching_robot_confirmation(
    monkeypatch, capsys
) -> None:
    monkeypatch.setattr(prrtc_demo, "load_execution_bundle", lambda *_a, **_k: _bundle())
    monkeypatch.setattr(
        prrtc_demo.WorldBaseTransform, "load", lambda *_a, **_k: object()
    )

    result = prrtc_demo.main(["bundle", "--execute"])

    assert result == 2
    assert "confirm-robot-ip" in capsys.readouterr().err


def test_physical_execution_uses_clean_current_checkouts_not_export_flag(
    monkeypatch, tmp_path: Path
) -> None:
    captured = {}

    def fake_load(*_args, **kwargs):
        captured["require_clean"] = kwargs["require_clean"]
        return _bundle()

    monkeypatch.setattr(prrtc_demo, "load_execution_bundle", fake_load)
    monkeypatch.setattr(
        prrtc_demo.WorldBaseTransform, "load", lambda *_a, **_k: object()
    )
    monkeypatch.setattr(
        prrtc_demo, "validate_physical_metadata", lambda *_a, **_k: None
    )
    monkeypatch.setattr(
        prrtc_demo,
        "git_state",
        lambda path: {"path": str(path), "commit": "abc", "dirty": False},
    )
    monkeypatch.setattr(
        prrtc_demo, "run_combined", lambda *_a, **_k: tmp_path
    )

    result = prrtc_demo.main(
        ["bundle", "--execute", "--confirm-robot-ip", "172.16.90.197"]
    )

    assert result == 0
    assert captured["require_clean"] is False


def test_combined_defaults_use_approved_ur_limits_and_gain() -> None:
    args = prrtc_demo._parser().parse_args(["bundle"])

    assert args.maximum_joint_speed_rad_s == pi
    assert args.maximum_joint_acceleration_rad_s2 == 40.0
    assert args.servo_gain == 1000.0
    assert args.first_joint_offset_rad == pi / 2.0
    assert args.last_joint_offset_rad == pi / 2.0


def test_clearance_model_undoes_physical_installation_offsets() -> None:
    physical = (
        0.2 + pi / 2.0,
        -1.0,
        1.2,
        -2.0,
        -1.5,
        0.3 + pi / 2.0,
    )

    planner = prrtc_demo.planner_joint_positions(
        physical, pi / 2.0, pi / 2.0
    )

    assert planner == pytest.approx((0.2, -1.0, 1.2, -2.0, -1.5, 0.3))


def test_maximum_arm_speed_resolves_shared_slowdown() -> None:
    main = JointTrajectory(
        "main",
        (JointSample(0.0, (0.0,) * 6), JointSample(1.0, (1.5,) * 6)),
    )
    park = JointTrajectory(
        "park",
        (JointSample(0.0, (1.5,) * 6), JointSample(1.0, (1.0,) * 6)),
    )
    preposition = JointTrajectory(
        "preposition",
        (JointSample(0.0, (0.0,) * 6), JointSample(1.0, (2.0,) * 6)),
    )
    bundle = SimpleNamespace(
        main=main, park=park, preposition=preposition
    )

    timescale = prrtc_demo.resolve_playback_timescale(
        bundle,
        playback_timescale=None,
        maximum_arm_speed_rad_s=0.5,
    )

    assert timescale == pytest.approx(4.0)


class _Flag:
    def __init__(self, value: bool = False) -> None:
        self.value = value

    def is_set(self) -> bool:
        return self.value


class _SequenceExecutor:
    def __init__(self) -> None:
        self.operations = []

    def execute(self, trajectory, start_at, *_args, **_kwargs):
        self.operations.append(("execute", trajectory.name, start_at))
        return ()

    def hold_until(self, target, deadline, *_args, **_kwargs):
        self.operations.append(("hold", tuple(target), deadline))
        return True


class _SequenceLog:
    def __init__(self) -> None:
        self.events = []

    def event(self, name, data) -> None:
        self.events.append((name, data))


def test_continuous_return_runs_at_goal_time_before_drone_endpoint() -> None:
    bundle = replace(_bundle(), return_during_drone_motion=True)
    executor = _SequenceExecutor()
    log = _SequenceLog()
    mission = SimpleNamespace(endpoint=_Flag(), failed=_Flag())

    prrtc_demo.complete_arm_mission_and_wait_for_drones(
        executor=executor,
        bundle=bundle,
        mission_node=mission,
        start_at=10.0,
        playback_timescale=1.0,
        record_sample=lambda _sample: None,
        log=log,
    )

    assert executor.operations[0] == ("execute", "park", 14.0)
    assert executor.operations[1][0:2] == ("hold", bundle.park.end)
    assert [name for name, _data in log.events] == [
        "arm_return_start",
        "arm_home_reached",
    ]


def test_legacy_return_still_waits_for_endpoint(monkeypatch) -> None:
    bundle = _bundle()
    executor = _SequenceExecutor()
    log = _SequenceLog()
    mission = SimpleNamespace(endpoint=_Flag(), failed=_Flag())
    monkeypatch.setattr(prrtc_demo.time, "monotonic", lambda: 20.0)

    prrtc_demo.complete_arm_mission_and_wait_for_drones(
        executor=executor,
        bundle=bundle,
        mission_node=mission,
        start_at=10.0,
        playback_timescale=1.0,
        record_sample=lambda _sample: None,
        log=log,
    )

    assert executor.operations[0][0:2] == ("hold", bundle.main.end)
    assert executor.operations[1] == ("execute", "park", 20.02)
    assert log.events == []
