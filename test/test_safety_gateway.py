import json
from pathlib import Path
from types import SimpleNamespace

from crazyfly.config import load_safety
from crazyfly.safety import Evaluation, SafetyAction, SafetyState
from crazyfly.safety_gateway import SafetyGateway, duration_message

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


def _batch_gateway(*, second_ready: bool = True):
    gateway = object.__new__(SafetyGateway)
    gateway.safety = load_safety(ROOT / "config" / "mock_safety.yaml")
    gateway.robot_names = ("cf1", "cf2")
    gateway._batch_end_at = None
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
                "frame": "world",
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
