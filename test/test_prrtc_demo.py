from types import SimpleNamespace
from math import pi

from crazyfly import prrtc_demo


def _bundle():
    return SimpleNamespace(
        bundle_id="bundle-1",
        robot_names=("cf1",),
        main=SimpleNamespace(duration_s=4.0),
        park=SimpleNamespace(duration_s=2.0),
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


def test_combined_defaults_use_approved_ur_limits_and_gain() -> None:
    args = prrtc_demo._parser().parse_args(["bundle"])

    assert args.maximum_joint_speed_rad_s == pi
    assert args.maximum_joint_acceleration_rad_s2 == 40.0
    assert args.servo_gain == 1000.0
