from types import SimpleNamespace

from crazyfly.safety import SafetyState
from crazyfly.safety_gateway import SafetyGateway, duration_message


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
