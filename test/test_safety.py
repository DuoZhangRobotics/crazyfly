from dataclasses import replace
from pathlib import Path

import pytest

from crazyfly.config import load_safety
from crazyfly.safety import (
    CAN_BE_ARMED,
    CAN_FLY,
    IS_ARMED,
    IS_CRASHED,
    IS_TUMBLED,
    SafetyAction,
    SafetyMachine,
    SafetyState,
    continuous_minimum_separation,
)


ROOT = Path(__file__).resolve().parents[1]
SUPERVISOR_READY = CAN_BE_ARMED | CAN_FLY


def _record_healthy(machine: SafetyMachine, now: float, positions=None) -> None:
    positions = positions or {
        name: (float(index), 0.0, 0.3)
        for index, name in enumerate(machine.robot_names)
    }
    for name in machine.robot_names:
        machine.record_pose(name, positions[name], now)
        machine.record_status(name, 4.1, SUPERVISOR_READY, now)


def _ready_machine(names=("cf1",)) -> SafetyMachine:
    machine = SafetyMachine(load_safety(ROOT / "config" / "mock_safety.yaml"), names)
    _record_healthy(machine, 0.0)
    machine.evaluate(0.0)
    _record_healthy(machine, 1.01)
    machine.evaluate(1.01)
    success, reasons = machine.enable(1.01)
    assert success, reasons
    return machine


def _ready_identity_machine(names=("cf1", "cf2")) -> SafetyMachine:
    config = replace(
        load_safety(ROOT / "config" / "mock_safety.yaml"),
        expected_raw_marker_count=len(names),
        marker_count_grace_s=0.05,
        pose_identity_emergency_age_s=0.10,
        maximum_pose_speed_m_s=0.75,
    )
    machine = SafetyMachine(config, names)
    _record_healthy(machine, 0.0)
    machine.record_raw_marker_count(len(names), 0.0)
    machine.evaluate(0.0)
    _record_healthy(machine, 1.01)
    machine.record_raw_marker_count(len(names), 1.01)
    machine.evaluate(1.01)
    success, reasons = machine.enable(1.01)
    assert success, reasons
    return machine


def _record_windowed_jump(
    machine: SafetyMachine,
    position: tuple[float, float, float],
    now: float = 1.07,
) -> None:
    machine.record_pose("cf1", (0.0, 0.0, 0.3), 1.035)
    machine.record_raw_marker_count(len(machine.robot_names), now)
    machine.record_status("cf1", 4.1, SUPERVISOR_READY, now)
    machine.record_pose("cf1", position, now)


def test_preflight_requires_a_stable_recovery_interval() -> None:
    machine = SafetyMachine(load_safety(ROOT / "config" / "mock_safety.yaml"), ("cf1",))
    _record_healthy(machine, 0.0)
    machine.evaluate(0.0)
    _record_healthy(machine, 0.5)

    success, reasons = machine.preflight(0.5)

    assert not success
    assert "healthy recovery interval has not completed" in reasons


def test_preflight_requires_the_reviewed_raw_marker_count() -> None:
    machine = _ready_identity_machine()
    machine.disable()
    machine.record_raw_marker_count(1, 2.0)
    _record_healthy(machine, 2.0)

    success, reasons = machine.preflight(2.0)

    assert not success
    assert "raw marker count mismatch: expected 2, got 1" in reasons


def test_persistent_raw_marker_loss_emergency_stops_before_reassignment() -> None:
    machine = _ready_identity_machine()
    machine.mark_flying()
    _record_healthy(machine, 1.02)
    machine.record_raw_marker_count(1, 1.02)

    transient = machine.evaluate(1.04)
    persistent = machine.evaluate(1.08)

    assert transient.action is SafetyAction.NONE
    assert transient.commands_allowed
    assert transient.reasons == ()
    assert persistent.action is SafetyAction.EMERGENCY
    assert persistent.reasons == ("raw marker count mismatch: expected 2, got 1",)


def test_identity_timeout_emergency_stops_instead_of_position_controlled_land() -> None:
    machine = _ready_identity_machine()
    machine.mark_flying()
    machine.record_raw_marker_count(2, 1.12)

    result = machine.evaluate(1.12)

    assert result.action is SafetyAction.EMERGENCY
    assert result.reasons == (
        "tracking identity lost: cf1",
        "tracking identity lost: cf2",
    )


def test_implausible_pose_jump_is_an_immediate_identity_fault() -> None:
    machine = _ready_identity_machine(("cf1",))
    machine.mark_flying()
    _record_windowed_jump(machine, (0.10, 0.0, 0.3))

    result = machine.evaluate(1.07)

    assert result.action is SafetyAction.EMERGENCY
    assert result.reasons[0].startswith("implausible pose speed: cf1")


def test_one_drone_pose_speed_wobble_requests_landing_without_lock() -> None:
    machine = _ready_identity_machine(("cf1",))
    machine.config = replace(machine.config, pose_speed_action="land")
    machine.mark_flying()
    _record_windowed_jump(machine, (0.10, 0.0, 0.3))

    result = machine.evaluate(1.07)

    assert result.action is SafetyAction.LAND
    assert result.state is SafetyState.LANDING
    assert machine.operator_enabled


def test_touchdown_pose_jump_does_not_emergency_stop_during_landing() -> None:
    machine = _ready_identity_machine(("cf1",))
    machine.mark_flying()
    machine.mark_landing()
    _record_windowed_jump(machine, (0.10, 0.0, 0.04))

    result = machine.evaluate(1.07)

    assert result.action is SafetyAction.NONE
    assert result.state is SafetyState.LANDING
    assert result.reasons[0].startswith("implausible pose speed: cf1")


def test_pose_speed_window_ignores_callback_arrival_jitter() -> None:
    machine = _ready_identity_machine(("cf1",))
    machine.config = replace(
        machine.config,
        maximum_pose_speed_m_s=1.5,
        pose_speed_window_s=0.05,
    )
    machine.mark_flying()
    received_at = 1.01
    arrival_deltas = [0.0083, 0.0084, 0.0082, 0.0084, 0.00423, 0.01243, 0.0083, 0.0083]
    for index, arrival_delta in enumerate(arrival_deltas, start=1):
        received_at += arrival_delta
        sample_time = 1.01 + index / 120.0
        position = (0.4 * (sample_time - 1.01), 0.0, 0.3)
        machine.record_pose(
            "cf1",
            position,
            received_at,
            sample_time,
        )
    machine.record_raw_marker_count(1, received_at)
    machine.record_status("cf1", 4.1, SUPERVISOR_READY, received_at)

    result = machine.evaluate(received_at)

    assert result.action is SafetyAction.NONE
    assert machine.health["cf1"].pose_speed_m_s == pytest.approx(0.4)
    assert machine.health["cf1"].pose_speed_fault_at is None


def test_pose_speed_window_detects_real_jump() -> None:
    machine = _ready_identity_machine(("cf1",))
    machine.config = replace(
        machine.config,
        maximum_pose_speed_m_s=1.5,
        pose_speed_window_s=0.05,
    )
    machine.mark_flying()
    _record_windowed_jump(machine, (0.10, 0.0, 0.3))

    result = machine.evaluate(1.07)

    assert result.action is SafetyAction.EMERGENCY
    assert machine.health["cf1"].pose_speed_fault_m_s == pytest.approx(
        0.10 / 0.06
    )


def test_pose_speed_history_resets_when_source_timestamp_moves_backward() -> None:
    machine = _ready_identity_machine(("cf1",))

    machine.record_pose("cf1", (0.5, 0.0, 0.3), 1.02, sample_time=0.5)

    assert list(machine.health["cf1"].pose_history) == [
        (0.5, (0.5, 0.0, 0.3))
    ]
    assert machine.health["cf1"].pose_speed_fault_at is None


def test_preflight_accepts_an_already_armed_auto_arm_crazyflie() -> None:
    machine = SafetyMachine(load_safety(ROOT / "config" / "mock_safety.yaml"), ("cf1",))
    supervisor_info = IS_ARMED | CAN_FLY
    machine.record_pose("cf1", (0.0, 0.0, 0.3), 0.0)
    machine.record_status("cf1", 4.1, supervisor_info, 0.0)
    machine.evaluate(0.0)
    machine.record_pose("cf1", (0.0, 0.0, 0.3), 1.01)
    machine.record_status("cf1", 4.1, supervisor_info, 1.01)
    machine.evaluate(1.01)

    success, reasons = machine.preflight(1.01)

    assert success, reasons


def test_tracking_loss_rejects_then_lands_then_emergency_stops() -> None:
    machine = _ready_machine()
    machine.mark_flying()

    rejected = machine.evaluate(1.12)
    landing = machine.evaluate(1.27)
    emergency = machine.evaluate(2.02)

    assert rejected.action is SafetyAction.NONE
    assert not rejected.commands_allowed
    assert landing.state is SafetyState.LANDING
    assert landing.action is SafetyAction.LAND
    assert emergency.state is SafetyState.EMERGENCY
    assert emergency.action is SafetyAction.EMERGENCY


def test_commands_wait_for_recovery_after_a_brief_tracking_fault() -> None:
    machine = _ready_machine()
    machine.mark_flying()
    assert not machine.evaluate(1.12).commands_allowed

    _record_healthy(machine, 1.13)
    assert not machine.evaluate(1.13).commands_allowed
    _record_healthy(machine, 2.14)
    assert machine.evaluate(2.14).commands_allowed


def test_battery_warning_does_not_block_commands_during_flight() -> None:
    machine = _ready_machine()
    machine.mark_flying()
    machine.record_pose("cf1", (0.0, 0.0, 0.3), 1.02)
    machine.record_status("cf1", 3.75, SUPERVISOR_READY, 1.02)

    result = machine.evaluate(1.02)

    assert result.action is SafetyAction.NONE
    assert result.state is SafetyState.FLYING
    assert result.commands_allowed


def test_battery_warning_blocks_preflight() -> None:
    machine = SafetyMachine(
        load_safety(ROOT / "config" / "mock_safety.yaml"), ("cf1",)
    )
    machine.record_pose("cf1", (0.0, 0.0, 0.0), 0.0)
    machine.record_status("cf1", 3.75, SUPERVISOR_READY, 0.0)

    success, reasons = machine.preflight(0.0)

    assert not success
    assert reasons == (
        "battery below threshold for cf1",
        "healthy recovery interval has not completed",
    )


def test_invalid_battery_requests_a_controlled_land() -> None:
    machine = _ready_machine()
    machine.mark_flying()
    machine.record_pose("cf1", (0.0, 0.0, 0.3), 1.02)
    machine.record_status("cf1", float("nan"), SUPERVISOR_READY, 1.02)

    result = machine.evaluate(1.02)

    assert result.action is SafetyAction.LAND
    assert result.state is SafetyState.LANDING


def test_soft_geofence_breach_requests_a_controlled_land() -> None:
    machine = _ready_machine()
    machine.mark_flying()
    machine.record_pose("cf1", (1.9, 0.0, 0.3), 1.02)
    machine.record_status("cf1", 4.1, SUPERVISOR_READY, 1.02)

    result = machine.evaluate(1.02)

    assert result.action is SafetyAction.LAND
    assert result.state is SafetyState.LANDING


def test_critical_battery_requests_one_controlled_land() -> None:
    machine = _ready_machine()
    machine.mark_flying()
    machine.record_pose("cf1", (0.0, 0.0, 0.3), 1.02)
    machine.record_status("cf1", 3.69, SUPERVISOR_READY, 1.02)

    first = machine.evaluate(1.02)
    second = machine.evaluate(1.03)

    assert first.action is SafetyAction.LAND
    assert first.state is SafetyState.LANDING
    assert second.action is SafetyAction.NONE


def test_loss_of_can_fly_is_an_immediate_hard_stop() -> None:
    machine = _ready_machine()
    machine.mark_flying()
    machine.record_pose("cf1", (0.0, 0.0, 0.3), 1.02)
    machine.record_status("cf1", 4.1, CAN_BE_ARMED, 1.02)

    result = machine.evaluate(1.02)

    assert result.action is SafetyAction.EMERGENCY
    assert result.state is SafetyState.EMERGENCY


def test_inactive_staged_drone_cannot_fly_does_not_stop_active_drone() -> None:
    machine = _ready_machine(("cf1", "cf2"))
    machine.mark_flying("cf1")
    machine.record_pose("cf1", (0.0, 0.0, 0.3), 1.02)
    machine.record_pose("cf2", (1.0, 0.0, 0.3), 1.02)
    machine.record_status("cf1", 4.1, SUPERVISOR_READY, 1.02)
    machine.record_status("cf2", 4.1, CAN_BE_ARMED, 1.02)

    result = machine.evaluate(1.02)

    assert result.action is SafetyAction.NONE
    assert result.state is SafetyState.FLYING
    assert "cannot fly: cf2" in result.reasons


def test_active_staged_drone_cannot_fly_is_an_emergency() -> None:
    machine = _ready_machine(("cf1", "cf2"))
    machine.mark_flying("cf1")
    machine.record_pose("cf1", (0.0, 0.0, 0.3), 1.02)
    machine.record_pose("cf2", (1.0, 0.0, 0.3), 1.02)
    machine.record_status("cf1", 4.1, CAN_BE_ARMED, 1.02)
    machine.record_status("cf2", 4.1, SUPERVISOR_READY, 1.02)

    result = machine.evaluate(1.02)

    assert result.action is SafetyAction.EMERGENCY
    assert result.reasons == ("cannot fly: cf1",)


def test_inactive_staged_drone_critical_battery_does_not_land_active() -> None:
    machine = _ready_machine(("cf1", "cf2"))
    machine.mark_flying("cf1")
    machine.record_pose("cf1", (0.0, 0.0, 0.3), 1.02)
    machine.record_pose("cf2", (1.0, 0.0, 0.3), 1.02)
    machine.record_status("cf1", 4.1, SUPERVISOR_READY, 1.02)
    machine.record_status("cf2", 3.0, SUPERVISOR_READY, 1.02)

    result = machine.evaluate(1.02)

    assert result.action is SafetyAction.NONE
    assert result.state is SafetyState.FLYING
    assert result.reasons == ()
    assert result.commands_allowed


def test_tumble_is_an_immediate_hard_stop() -> None:
    machine = _ready_machine()
    machine.mark_flying()
    machine.record_status("cf1", 4.1, SUPERVISOR_READY | IS_TUMBLED, 1.02)

    result = machine.evaluate(1.02)

    assert result.action is SafetyAction.EMERGENCY
    assert result.state is SafetyState.EMERGENCY


def test_crash_is_an_immediate_hard_stop() -> None:
    machine = _ready_machine()
    machine.mark_flying()
    machine.record_status("cf1", 4.1, SUPERVISOR_READY | IS_CRASHED, 1.02)

    result = machine.evaluate(1.02)

    assert result.action is SafetyAction.EMERGENCY
    assert result.state is SafetyState.EMERGENCY


def test_command_limits_and_separation_are_enforced() -> None:
    positions = {"cf1": (0.0, 0.0, 0.3), "cf2": (1.0, 0.0, 0.3)}
    machine = SafetyMachine(load_safety(ROOT / "config" / "mock_safety.yaml"), ("cf1", "cf2"))
    _record_healthy(machine, 0.0, positions)
    machine.evaluate(0.0)
    _record_healthy(machine, 1.01, positions)
    machine.evaluate(1.01)
    assert machine.enable(1.01)[0]

    assert not machine.validate_takeoff("cf1", 0.6, 3.0)[0]
    assert not machine.validate_takeoff("cf1", 0.5, 0.5)[0]
    assert not machine.validate_takeoff("cf1", 0.1, 2.0)[0]
    assert machine.validate_takeoff("cf1", 0.3, 2.0)[0]
    machine.mark_flying("cf1")
    assert not machine.validate_takeoff("cf1", 0.3, 2.0)[0]
    assert machine.validate_takeoff("cf2", 0.3, 2.0)[0]

    machine.mark_flying()
    assert not machine.validate_goto("cf1", (3.0, 0.0, 0.3), 20.0)[0]
    assert not machine.validate_goto("cf1", (0.5, 0.0, 0.3), 1.0)[0]
    assert "separation" in machine.validate_goto("cf1", (0.7, 0.0, 0.3), 3.0)[1]
    assert not machine.validate_goto("cf1", (float("nan"), 0.0, 0.3), 1.0)[0]
    assert not machine.validate_goto("cf1", (0.1, 0.0, 0.3), float("nan"))[0]
    assert machine.validate_goto("cf1", (0.1, 0.0, 0.3), 1.0)[0]


def _coordinated_square_machine() -> SafetyMachine:
    names = ("cf1", "cf2", "cf3", "cf4")
    positions = {
        "cf1": (0.4, 0.4, 0.3),
        "cf2": (-0.4, 0.4, 0.3),
        "cf3": (-0.4, -0.4, 0.3),
        "cf4": (0.4, -0.4, 0.3),
    }
    config = replace(
        load_safety(ROOT / "config" / "mock_safety.yaml"),
        minimum_separation_m=0.15,
    )
    machine = SafetyMachine(config, names)
    _record_healthy(machine, 0.0, positions)
    machine.evaluate(0.0)
    _record_healthy(machine, 1.01, positions)
    machine.evaluate(1.01)
    assert machine.enable(1.01)[0]
    for name in names:
        machine.mark_flying(name)
    return machine


def test_coordinated_cycle_validates_continuous_separation() -> None:
    machine = _coordinated_square_machine()
    names = machine.robot_names
    starts = {name: machine.health[name].position for name in names}
    goals = {
        name: starts[names[(index + 1) % len(names)]]
        for index, name in enumerate(names)
    }

    accepted, reason, minimum = machine.validate_coordinated_goto(goals, 4.0)

    assert accepted, reason
    assert minimum == pytest.approx(2**0.5 * 0.4)
    assert not machine.validate_goto("cf1", goals["cf1"], 4.0)[0]


def test_coordinated_head_on_swap_is_rejected_between_endpoints() -> None:
    starts = {"cf1": (-0.5, 0.0, 0.3), "cf2": (0.5, 0.0, 0.3)}
    goals = {"cf1": starts["cf2"], "cf2": starts["cf1"]}

    minimum, pair, fraction = continuous_minimum_separation(starts, goals)

    assert minimum == pytest.approx(0.0)
    assert pair == ("cf1", "cf2")
    assert fraction == pytest.approx(0.5)


def test_coordinated_translation_preserves_separation() -> None:
    starts = {"cf1": (0.0, 0.0, 0.3), "cf2": (0.4, 0.0, 0.3)}
    goals = {"cf1": (0.08, 0.0, 0.3), "cf2": (0.48, 0.0, 0.3)}

    minimum, pair, _fraction = continuous_minimum_separation(starts, goals)

    assert minimum == pytest.approx(0.4)
    assert pair == ("cf1", "cf2")


def test_single_robot_coordinated_path_has_infinite_separation() -> None:
    minimum, pair, fraction = continuous_minimum_separation(
        {"cf1": (0.0, 0.0, 0.3)},
        {"cf1": (0.08, 0.0, 0.3)},
    )

    assert minimum == float("inf")
    assert pair == ("cf1", "cf1")
    assert fraction == 0.0
