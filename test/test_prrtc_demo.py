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


def test_clearance_model_undoes_physical_first_joint_offset() -> None:
    physical = (0.2 + pi / 2.0, -1.0, 1.2, -2.0, -1.5, 0.3)

    planner = prrtc_demo.planner_joint_positions(physical, pi / 2.0)

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
    bundle = SimpleNamespace(main=main, park=park)

    timescale = prrtc_demo.resolve_playback_timescale(
        bundle,
        playback_timescale=None,
        maximum_arm_speed_rad_s=0.5,
    )

    assert timescale == pytest.approx(3.0)
