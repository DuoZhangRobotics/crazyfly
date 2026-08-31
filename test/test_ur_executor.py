from dataclasses import replace

import pytest

from crazyfly.prrtc_bundle import JointSample, JointTrajectory
from crazyfly.ur_executor import TimedURExecutor, URExecutionConfig


class Clock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, duration: float) -> None:
        self.now += duration


class Receive:
    def __init__(self) -> None:
        self.q = [0.0] * 6
        self.offset = 0.0
        self.disconnected = False

    def getActualQ(self):
        return [value + self.offset for value in self.q]

    def getActualTCPPose(self):
        return [0.0] * 6

    def getRobotMode(self):
        return 7

    def getSafetyMode(self):
        return 1

    def disconnect(self):
        self.disconnected = True


class Control:
    def __init__(self, receive: Receive, clock: Clock) -> None:
        self.receive = receive
        self.clock = clock
        self.stopped = []
        self.disconnected = False
        self.servo_parameters = []

    def moveJ(self, q, _speed, _acceleration):
        self.receive.q = list(q)
        return True

    def servoJ(self, q, _speed, _acceleration, period, lookahead, gain):
        self.servo_parameters.append((period, lookahead, gain))
        self.receive.q = list(q)
        return True

    def initPeriod(self):
        return object()

    def waitPeriod(self, _token):
        self.clock.sleep(0.01)

    def servoStop(self, acceleration=10.0):
        self.stopped.append(("servoStop", acceleration))
        return True

    def stopJ(self, acceleration=2.0, asynchronous=False):
        self.stopped.append(("stopJ", acceleration, asynchronous))

    def disconnect(self):
        self.disconnected = True


def trajectory(duration: float = 0.03) -> JointTrajectory:
    return JointTrajectory(
        "test",
        (
            JointSample(0.0, (0.0,) * 6),
            JointSample(duration, (0.1,) * 6),
        ),
    )


def executor(config: URExecutionConfig = URExecutionConfig()):
    clock = Clock()
    receive = Receive()
    control = Control(receive, clock)
    return TimedURExecutor(
        control, receive, config, clock=clock, sleep=clock.sleep
    ), control, receive, clock


def test_executor_prepositions_and_runs_on_scheduled_timeline() -> None:
    item, control, receive, clock = executor(
        replace(URExecutionConfig(), ready_stable_s=0.02)
    )
    item.validate_robot_ready()
    item.preposition((0.0,) * 6)

    samples = item.execute(trajectory(), start_at=1.0)

    assert clock.now >= 1.03
    assert samples[0].elapsed_s == pytest.approx(0.0)
    assert samples[-1].commanded_rad == pytest.approx((0.1,) * 6)
    assert receive.q == pytest.approx([0.1] * 6)
    assert control.servo_parameters
    assert control.servo_parameters[0] == pytest.approx((0.01, 0.1, 1000.0))


def test_executor_aborts_after_three_joint_error_cycles() -> None:
    item, _control, receive, _clock = executor()
    receive.offset = 0.1

    with pytest.raises(RuntimeError, match="joint error"):
        item.execute(trajectory(), start_at=0.0)


def test_executor_cleanup_stops_and_disconnects_both_interfaces() -> None:
    item, control, receive, _clock = executor()

    item.stop()
    item.disconnect()

    assert control.stopped == [("servoStop", 2.0), ("stopJ", 1.0, False)]
    assert control.disconnected
    assert receive.disconnected
