import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from crazyfly.config import load_safety
from crazyfly.safety import Evaluation, SafetyAction, SafetyState
from crazyfly.safety_gateway import (
    PreparedTrajectory,
    SafetyGateway,
    duration_message,
)
from crazyfly.trajectory import load_trajectory, trajectory_payload

ROOT = Path(__file__).resolve().parents[1]


def _gateway(state: SafetyState, operator_enabled: bool):
    gateway = object.__new__(SafetyGateway)
    gateway.machine = SimpleNamespace(state=state, operator_enabled=operator_enabled)
    gateway.machine.mark_landing = lambda: setattr(
        gateway.machine, "state", SafetyState.LANDING
    )
    gateway.rejections = []
    gateway.forwards = []
    gateway._reject = lambda command, reason: gateway.rejections.append(
        (command, reason)
    )
    gateway._forward = lambda name, kind, request: (
        gateway.forwards.append((name, kind)) or True
    )
    gateway._publish_command = lambda *args, **kwargs: None
    gateway._now = lambda: 10.0
    gateway._landing_disarm_at = None
    gateway._landing_robots = set()
    return gateway


def _request(duration_s: float):
    return SimpleNamespace(height=0.04, duration=duration_message(duration_s))


def test_pose_callback_uses_message_stamp_for_speed_and_receipt_for_age() -> None:
    gateway = object.__new__(SafetyGateway)
    calls = []
    gateway.machine = SimpleNamespace(
        record_pose=lambda *arguments: calls.append(arguments)
    )
    gateway._now = lambda: 10.0
    message = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=123, nanosec=500_000_000)),
        poses=[
            SimpleNamespace(
                name="cf1",
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=1.0, y=2.0, z=3.0)
                ),
            )
        ],
    )

    gateway._pose_callback(message)

    assert calls == [("cf1", (1.0, 2.0, 3.0), 10.0, 123.5)]


def test_land_is_rejected_after_gateway_disables() -> None:
    gateway = _gateway(SafetyState.DISABLED, False)
    response = SimpleNamespace()
    assert gateway._land_callback("cf1", _request(1.0), response) is response
    assert gateway.rejections == [("land/cf1", "commands are blocked in DISABLED")]
    assert gateway.forwards == []


def test_land_disarms_after_requested_duration_plus_margin() -> None:
    gateway = _gateway(SafetyState.FLYING, True)
    response = SimpleNamespace()
    assert gateway._land_callback("cf1", _request(3.0), response) is response
    assert gateway.forwards == [("cf1", "land")]
    assert gateway.machine.state is SafetyState.LANDING
    assert gateway._landing_disarm_at == 13.25


def test_duplicate_land_is_rejected_while_landing() -> None:
    gateway = _gateway(SafetyState.LANDING, True)
    gateway._landing_robots.add("cf1")
    response = SimpleNamespace()
    assert gateway._land_callback("cf1", _request(1.0), response) is response
    assert gateway.rejections == [
        ("land/cf1", "landing is already in progress for cf1")
    ]
    assert gateway.forwards == []


def test_distinct_robots_can_receive_synchronized_land_requests() -> None:
    gateway = _gateway(SafetyState.FLYING, True)
    response = SimpleNamespace()

    assert gateway._land_callback("cf1", _request(1.0), response) is response
    gateway._now = lambda: 10.1
    assert gateway._land_callback("cf2", _request(1.5), response) is response

    assert gateway.forwards == [("cf1", "land"), ("cf2", "land")]
    assert gateway.rejections == []
    assert gateway._landing_robots == {"cf1", "cf2"}
    assert gateway._landing_disarm_at == 11.85


class _BatchClient:
    def __init__(self, ready: bool = True):
        self.ready = ready
        self.requests = []

    def service_is_ready(self) -> bool:
        return self.ready

    def call_async(self, request) -> None:
        self.requests.append(request)


class _Future:
    def __init__(self, error: Exception | None = None):
        self.error = error

    def done(self) -> bool:
        return True

    def result(self):
        if self.error is not None:
            raise self.error
        return SimpleNamespace()


class _TrajectoryClient(_BatchClient):
    def __init__(self, ready: bool = True, error: Exception | None = None):
        super().__init__(ready)
        self.error = error

    def call_async(self, request):
        self.requests.append(request)
        return _Future(self.error)


def _batch_gateway(*, second_ready: bool = True):
    gateway = object.__new__(SafetyGateway)
    gateway.safety = load_safety(ROOT / "config" / "mock_safety.yaml")
    gateway.robot_names = ("cf1", "cf2")
    gateway._batch_end_at = None
    gateway._trajectory_end_at = None
    gateway._now = lambda: 10.0
    gateway.last_evaluation = Evaluation(
        SafetyState.FLYING, SafetyAction.NONE, (), True
    )
    gateway.machine = SimpleNamespace(
        state=SafetyState.FLYING,
        validate_coordinated_goto=lambda goals, duration: (
            True,
            "accepted",
            0.4,
        ),
    )
    first = _BatchClient()
    second = _BatchClient(second_ready)
    gateway.upstream_clients = {
        "cf1": {"go_to": first},
        "cf2": {"go_to": second},
    }
    gateway.events = []
    gateway._publish_command = lambda *args, **kwargs: gateway.events.append(
        (args, kwargs)
    )
    gateway.get_logger = lambda: SimpleNamespace(
        info=lambda _message: None,
        warning=lambda _message: None,
    )
    return gateway, first, second


def _batch_message():
    return SimpleNamespace(
        data=json.dumps(
            {
                "batch_id": "test-1",
                "frame": "base",
                "duration_s": 2.0,
                "yaw_rad": 0.0,
                "goals": {
                    "cf1": [0.1, 0.0, 0.3],
                    "cf2": [0.5, 0.0, 0.3],
                },
            }
        )
    )


def test_batch_go_to_dispatches_all_requests_after_atomic_validation() -> None:
    gateway, first, second = _batch_gateway()

    gateway._batch_goto_callback(_batch_message())

    assert len(first.requests) == 1
    assert len(second.requests) == 1
    assert gateway._batch_end_at == 12.0
    assert gateway.events[-1][0][:2] == ("batch_go_to", "accepted")


def test_batch_go_to_dispatches_none_if_any_service_is_unavailable() -> None:
    gateway, first, second = _batch_gateway(second_ready=False)

    gateway._batch_goto_callback(_batch_message())

    assert first.requests == []
    assert second.requests == []
    assert gateway._batch_end_at is None
    assert gateway.events[-1][0][:2] == ("batch_go_to", "rejected")


def _trajectory_gateway(*, upload_error: Exception | None = None):
    gateway = object.__new__(SafetyGateway)
    gateway.safety = load_safety(ROOT / "config" / "mock_safety.yaml")
    gateway.robot_names = ("cf1",)
    gateway.machine = SimpleNamespace(
        state=SafetyState.DISABLED,
        operator_enabled=False,
        health={"cf1": SimpleNamespace(position=(0.0, 0.0, 0.3))},
    )
    gateway.last_evaluation = Evaluation(
        SafetyState.DISABLED, SafetyAction.NONE, (), False
    )
    gateway._trajectory_upload_pending = None
    gateway._prepared_trajectory = None
    gateway._trajectory_end_at = None
    gateway._trajectory_started = False
    gateway._batch_end_at = None
    gateway._last_rejection = ""
    gateway._now = lambda: 10.0
    upload = _TrajectoryClient(error=upload_error)
    gateway.upstream_clients = {"cf1": {"upload_trajectory": upload}}
    gateway.start_trajectory_client = _TrajectoryClient()
    gateway.events = []
    gateway._publish_command = lambda *args, **kwargs: gateway.events.append(
        (args, kwargs)
    )
    gateway.get_logger = lambda: SimpleNamespace(
        info=lambda _message: None,
        warning=lambda _message: None,
    )
    fleet = __import__("crazyfly.config", fromlist=["load_fleet"]).load_fleet(
        ROOT / "config" / "mock_crazyflies.yaml"
    )
    plan = load_trajectory(
        ROOT / "config" / "trajectories" / "mock_square.yaml",
        fleet,
        gateway.safety,
    )
    payload = trajectory_payload(plan, mission_id="mission-1", trajectory_id=3)
    message = SimpleNamespace(data=json.dumps(payload))
    return gateway, upload, plan, message


def test_trajectory_upload_is_prepared_only_after_all_futures_finish() -> None:
    gateway, upload, plan, message = _trajectory_gateway()

    gateway._trajectory_upload_callback(message)

    assert len(upload.requests) == 1
    assert gateway._prepared_trajectory is None
    assert gateway.events[-1][0][:2] == ("trajectory_upload", "pending")

    gateway._poll_trajectory_upload()

    assert gateway._prepared_trajectory is not None
    assert gateway._prepared_trajectory.mission_id == "mission-1"
    assert gateway._prepared_trajectory.trajectory_id == 3
    assert gateway._prepared_trajectory.plan.trajectories == plan.trajectories
    assert gateway.events[-1][0][:2] == ("trajectory_upload", "accepted")
    request = upload.requests[0]
    assert request.trajectory_id == 3
    assert request.piece_offset == 0
    assert len(request.pieces) == 4


def test_failed_trajectory_upload_never_becomes_startable() -> None:
    gateway, upload, _plan, message = _trajectory_gateway(
        upload_error=RuntimeError("upload failed")
    )

    gateway._trajectory_upload_callback(message)
    gateway._poll_trajectory_upload()

    assert len(upload.requests) == 1
    assert gateway._prepared_trajectory is None
    assert gateway.events[-1][0][:2] == ("trajectory_upload", "rejected")


def test_trajectory_upload_requires_disarmed_state_and_ready_services() -> None:
    gateway, upload, _plan, message = _trajectory_gateway()
    gateway.machine.operator_enabled = True
    gateway.machine.state = SafetyState.READY

    gateway._trajectory_upload_callback(message)

    assert upload.requests == []
    assert "requires disarmed" in gateway._last_rejection

    gateway.machine.operator_enabled = False
    gateway.machine.state = SafetyState.DISABLED
    upload.ready = False
    gateway._trajectory_upload_callback(message)
    assert upload.requests == []
    assert "unavailable" in gateway._last_rejection


def _start_message(mission_id="mission-1", trajectory_id=3, timescale=1.0):
    return SimpleNamespace(
        data=json.dumps(
            {
                "mission_id": mission_id,
                "trajectory_id": trajectory_id,
                "timescale": timescale,
            }
        )
    )


def test_trajectory_start_requires_matching_prepared_mission_and_position() -> None:
    gateway, _upload, plan, _message = _trajectory_gateway()
    gateway._prepared_trajectory = PreparedTrajectory("mission-1", 3, plan)
    gateway.machine.state = SafetyState.FLYING
    gateway.machine.operator_enabled = True
    gateway.machine.health["cf1"].position = plan.start_positions["cf1"]
    gateway.last_evaluation = Evaluation(
        SafetyState.FLYING, SafetyAction.NONE, (), True
    )

    gateway._trajectory_start_callback(_start_message())

    assert len(gateway.start_trajectory_client.requests) == 1
    request = gateway.start_trajectory_client.requests[0]
    assert not request.relative
    assert not request.reversed
    assert request.timescale == 1.0
    assert gateway._trajectory_started
    assert gateway._trajectory_end_at == 10.0 + plan.duration_s
    assert gateway.events[-1][0][:2] == ("trajectory_start", "accepted")

    gateway._trajectory_start_callback(_start_message())
    assert len(gateway.start_trajectory_client.requests) == 1
    assert gateway.events[-1][0][:2] == ("trajectory_start", "rejected")


def test_trajectory_start_propagates_safe_slowdown() -> None:
    gateway, _upload, plan, _message = _trajectory_gateway()
    gateway._prepared_trajectory = PreparedTrajectory("mission-1", 3, plan)
    gateway.machine.state = SafetyState.FLYING
    gateway.machine.operator_enabled = True
    gateway.machine.health["cf1"].position = plan.start_positions["cf1"]
    gateway.last_evaluation = Evaluation(
        SafetyState.FLYING, SafetyAction.NONE, (), True
    )

    gateway._trajectory_start_callback(_start_message(timescale=2.5))

    request = gateway.start_trajectory_client.requests[0]
    assert request.timescale == pytest.approx(2.5)
    assert gateway._trajectory_end_at == pytest.approx(
        10.0 + plan.duration_s * 2.5
    )
    details = json.loads(gateway.events[-1][1]["details"])
    assert details["duration_s"] == pytest.approx(plan.duration_s * 2.5)
    assert details["timescale"] == pytest.approx(2.5)


def test_trajectory_start_rejects_speedup_timescale() -> None:
    gateway, _upload, plan, _message = _trajectory_gateway()
    gateway._prepared_trajectory = PreparedTrajectory("mission-1", 3, plan)

    gateway._trajectory_start_callback(_start_message(timescale=0.5))

    assert gateway.start_trajectory_client.requests == []
    assert "at least 1.0" in gateway._last_rejection


def test_trajectory_start_rejects_wrong_id_and_unsettled_start() -> None:
    gateway, _upload, plan, _message = _trajectory_gateway()
    gateway._prepared_trajectory = PreparedTrajectory("mission-1", 3, plan)
    gateway.machine.state = SafetyState.FLYING
    gateway.machine.operator_enabled = True
    gateway.last_evaluation = Evaluation(
        SafetyState.FLYING, SafetyAction.NONE, (), True
    )

    gateway._trajectory_start_callback(_start_message(trajectory_id=4))
    assert gateway.start_trajectory_client.requests == []
    assert "mismatch" in gateway._last_rejection

    gateway.machine.health["cf1"].position = (0.2, 0.0, 0.3)
    gateway._trajectory_start_callback(_start_message())
    assert gateway.start_trajectory_client.requests == []
    assert "not settled" in gateway._last_rejection
