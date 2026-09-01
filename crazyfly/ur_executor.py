"""Timed UR5e joint execution with RTDE watchdogs and injectable fakes."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Callable, Protocol, Sequence

from .config import ConfigError
from .prrtc_bundle import JointTrajectory


class Control(Protocol):
    def moveJ(self, q, speed, acceleration) -> bool: ...
    def servoJ(self, q, speed, acceleration, period, lookahead, gain) -> bool: ...
    def initPeriod(self): ...
    def waitPeriod(self, token) -> None: ...
    def servoStop(self, acceleration=10.0) -> bool: ...
    def stopJ(self, acceleration=2.0, asynchronous=False) -> None: ...
    def disconnect(self) -> None: ...


class Receive(Protocol):
    def getActualQ(self) -> Sequence[float]: ...
    def getActualTCPPose(self) -> Sequence[float]: ...
    def getRobotMode(self) -> int: ...
    def getSafetyMode(self) -> int: ...
    def disconnect(self) -> None: ...


@dataclass(frozen=True)
class URExecutionConfig:
    frequency_hz: float = 100.0
    move_speed_rad_s: float = 0.25
    move_acceleration_rad_s2: float = 0.25
    servo_lookahead_s: float = 0.03
    servo_gain: float = 1000.0
    ready_tolerance_rad: float = 0.03
    ready_stable_s: float = 0.5
    maximum_joint_error_rad: float = 0.087
    error_consecutive_cycles: int = 3
    maximum_control_gap_s: float = 0.05
    servo_stop_acceleration: float = 2.0
    stop_joint_acceleration: float = 1.0

    @property
    def period_s(self) -> float:
        return 1.0 / self.frequency_hz


@dataclass(frozen=True)
class URSample:
    monotonic_s: float
    elapsed_s: float
    commanded_rad: tuple[float, ...]
    actual_rad: tuple[float, ...]
    tcp_pose: tuple[float, ...]
    maximum_error_rad: float


class TimedURExecutor:
    def __init__(
        self,
        control: Control,
        receive: Receive,
        config: URExecutionConfig = URExecutionConfig(),
        *,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.control = control
        self.receive = receive
        self.config = config
        self.clock = clock
        self.sleep = sleep

    def validate_robot_ready(self) -> None:
        if self.receive.getRobotMode() != 7:
            raise RuntimeError("UR5e robot mode is not RUNNING")
        if self.receive.getSafetyMode() != 1:
            raise RuntimeError("UR5e safety mode is not NORMAL")
        self._actual_q()

    def _actual_q(self) -> tuple[float, ...]:
        values = tuple(float(value) for value in self.receive.getActualQ())
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            raise RuntimeError("UR5e returned invalid joint positions")
        return values

    def preposition(self, target: Sequence[float], timeout_s: float = 30.0) -> None:
        if not self.control.moveJ(
            list(target),
            self.config.move_speed_rad_s,
            self.config.move_acceleration_rad_s2,
        ):
            raise RuntimeError("UR5e moveJ preposition failed")
        stable_since = None
        deadline = self.clock() + timeout_s
        while self.clock() < deadline:
            error = max(abs(a - b) for a, b in zip(self._actual_q(), target))
            if error <= self.config.ready_tolerance_rad:
                stable_since = stable_since or self.clock()
                if self.clock() - stable_since >= self.config.ready_stable_s:
                    return
            else:
                stable_since = None
            self.sleep(self.config.period_s)
        raise RuntimeError("UR5e did not settle at the trajectory start")

    def execute(
        self,
        trajectory: JointTrajectory,
        start_at: float,
        on_sample: Callable[[URSample], None] | None = None,
        stop_when: Callable[[], bool] = lambda: False,
    ) -> tuple[URSample, ...]:
        while self.clock() < start_at:
            self.sleep(min(self.config.period_s, start_at - self.clock()))
        samples = []
        error_cycles = 0
        previous_cycle = self.clock()
        while True:
            if stop_when():
                raise RuntimeError("UR5e execution stopped by coordinated fault")
            cycle_started = self.clock()
            gap = cycle_started - previous_cycle
            if samples and gap > self.config.maximum_control_gap_s:
                raise RuntimeError(f"UR5e control gap exceeded limit: {gap:.3f} s")
            previous_cycle = cycle_started
            elapsed = max(0.0, cycle_started - start_at)
            command = trajectory.evaluate(elapsed)
            token = self.control.initPeriod()
            if not self.control.servoJ(
                list(command),
                0.0,
                0.0,
                self.config.period_s,
                self.config.servo_lookahead_s,
                self.config.servo_gain,
            ):
                raise RuntimeError("UR5e servoJ command failed")
            actual = self._actual_q()
            tcp = tuple(float(value) for value in self.receive.getActualTCPPose())
            error = max(abs(a - b) for a, b in zip(actual, command))
            error_cycles = error_cycles + 1 if error > self.config.maximum_joint_error_rad else 0
            sample = URSample(
                monotonic_s=cycle_started,
                elapsed_s=min(elapsed, trajectory.duration_s),
                commanded_rad=command,
                actual_rad=actual,
                tcp_pose=tcp,
                maximum_error_rad=error,
            )
            samples.append(sample)
            if on_sample is not None:
                on_sample(sample)
            if error_cycles >= self.config.error_consecutive_cycles:
                raise RuntimeError(f"UR5e joint error exceeded limit: {error:.3f} rad")
            self.control.waitPeriod(token)
            if elapsed >= trajectory.duration_s:
                return tuple(samples)

    def hold_until(
        self,
        target: Sequence[float],
        deadline: float,
        stop_when: Callable[[], bool] = lambda: False,
        on_sample: Callable[[URSample], None] | None = None,
    ) -> bool:
        while self.clock() < deadline and not stop_when():
            cycle_started = self.clock()
            token = self.control.initPeriod()
            if not self.control.servoJ(
                list(target), 0.0, 0.0, self.config.period_s,
                self.config.servo_lookahead_s, self.config.servo_gain,
            ):
                raise RuntimeError("UR5e goal hold failed")
            actual = self._actual_q()
            command = tuple(float(value) for value in target)
            if on_sample is not None:
                on_sample(URSample(
                    monotonic_s=cycle_started,
                    elapsed_s=0.0,
                    commanded_rad=command,
                    actual_rad=actual,
                    tcp_pose=tuple(
                        float(value) for value in self.receive.getActualTCPPose()
                    ),
                    maximum_error_rad=max(
                        abs(first - second)
                        for first, second in zip(actual, command)
                    ),
                ))
            self.control.waitPeriod(token)
        return stop_when()

    def stop(self) -> None:
        try:
            self.control.servoStop(self.config.servo_stop_acceleration)
        finally:
            self.control.stopJ(self.config.stop_joint_acceleration)

    def disconnect(self) -> None:
        self.control.disconnect()
        self.receive.disconnect()


def connect_ur5e(robot_ip: str, frequency_hz: float = 100.0):
    try:
        from rtde_control import RTDEControlInterface
        from rtde_receive import RTDEReceiveInterface
    except ImportError as exc:
        raise ConfigError("ur_rtde is not installed in the active environment") from exc
    return (
        RTDEControlInterface(robot_ip, frequency_hz),
        RTDEReceiveInterface(
            robot_ip, frequency_hz, use_upper_range_registers=False
        ),
    )
