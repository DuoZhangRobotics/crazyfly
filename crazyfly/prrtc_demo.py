"""One-command scheduled UR5e and Crazyflie pRRTC execution."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import threading
import time
from contextlib import suppress
from dataclasses import asdict, replace
from math import pi

from ur_tools.optitrack.frame_transform import WorldBaseTransform

from .config import ConfigError, load_safety
from .prrtc_bundle import (
    ExecutionBundle,
    load_execution_bundle,
    maximum_joint_speed,
    scale_trajectory_time,
    validate_physical_metadata,
)
from .trajectory import evaluate_plan, trajectory_from_payload
from .ur_executor import (
    TimedURExecutor,
    URExecutionConfig,
    URSample,
    connect_ur5e,
)


ROOT = Path(__file__).resolve().parents[1]
UR_TOOLS_ROOT = Path("/home/duo/ur_tools")
PRRTC_ROOT = Path("/home/duo/pRRTC")
DEFAULT_CALIBRATION = UR_TOOLS_ROOT / "config" / "optitrack_to_ur_base.yaml"
DEFAULT_OUTPUT_ROOT = ROOT / "combined_experiments"
DEFAULT_SAFETY = ROOT / "config" / "local" / "safety.yaml"


def git_state(path: Path) -> dict[str, object]:
    commit = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=path, check=True,
        capture_output=True, text=True,
    ).stdout.strip()
    dirty = bool(subprocess.run(
        ["git", "status", "--porcelain"], cwd=path, check=True,
        capture_output=True, text=True,
    ).stdout.strip())
    return {"path": str(path), "commit": commit, "dirty": dirty}


def file_record(path: Path) -> dict[str, str]:
    resolved = path.resolve()
    return {
        "path": str(resolved),
        "sha256": hashlib.sha256(resolved.read_bytes()).hexdigest(),
    }


class CombinedLog:
    def __init__(self, output_root: Path, bundle: ExecutionBundle) -> None:
        run_id = time.strftime("%Y%m%dT%H%M%S", time.gmtime())
        run_id += f"-{time.time_ns() % 1_000_000_000:09d}"
        self.root = output_root.resolve() / run_id
        self.root.mkdir(parents=True)
        shutil.copytree(bundle.root, self.root / "execution_bundle")
        self.arm_file = (self.root / "ur_samples.jsonl").open(
            "w", encoding="utf-8"
        )
        self.event_file = (self.root / "events.jsonl").open(
            "w", encoding="utf-8"
        )
        self.arm_samples: list[URSample] = []
        self.hardware_samples: list[dict[str, object]] = []
        self.bundle = bundle
        mapping = bundle.manifest.get("name_mapping", {})
        self.name_mapping = dict(mapping) if isinstance(mapping, dict) else {}
        self.manifest = {
            "run_id": run_id,
            "bundle_id": bundle.bundle_id,
            "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "repositories": {
                "crazyfly": git_state(ROOT),
                "ur_tools": git_state(UR_TOOLS_ROOT),
                "pRRTC": {
                    "bundle_source": bundle.manifest.get("pRRTC_source")
                    or bundle.manifest.get("pRRTC"),
                    "current_checkout": git_state(PRRTC_ROOT),
                },
            },
            "calibration": file_record(DEFAULT_CALIBRATION),
            "status": "incomplete",
        }
        self._write_manifest()

    def _write_manifest(self) -> None:
        (self.root / "manifest.json").write_text(
            json.dumps(self.manifest, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    def event(self, name: str, data: object) -> None:
        self.event_file.write(json.dumps({
            "monotonic_s": time.monotonic(), "event": name, "data": data,
        }, separators=(",", ":")) + "\n")
        self.event_file.flush()

    def arm_sample(self, sample: URSample) -> None:
        self.arm_samples.append(sample)
        self.arm_file.write(json.dumps(asdict(sample), separators=(",", ":")) + "\n")
        self.arm_file.flush()

    def hardware_sample(
        self,
        sample: URSample,
        commanded_drones: dict[str, tuple[float, float, float]],
        measured_drones: dict[str, tuple[float, float, float]],
        separations: dict[str, float],
    ) -> None:
        self.hardware_samples.append({
            "monotonic_time_s": sample.monotonic_s,
            "clock_skew_s": 0.0,
            "commanded_ur5e_joints_rad": list(sample.commanded_rad),
            "measured_ur5e_joints_rad": list(sample.actual_rad),
            "drones": [
                {
                    "name": name,
                    "commanded_position_m": list(commanded_drones[name]),
                    "measured_position_m": list(measured_drones[name]),
                    "measured_center_to_robot_separation_m": separations[name],
                }
                for name in sorted(commanded_drones)
                if name in measured_drones and name in separations
            ],
        })
        for drone in self.hardware_samples[-1]["drones"]:
            drone["name"] = self.name_mapping.get(drone["name"], drone["name"])

    def finish(self, *, success: bool, start_skew_s: float | None, reason: str = "") -> None:
        errors = [sample.maximum_error_rad for sample in self.arm_samples]
        metrics = {
            "maximum_joint_error_rad": max(errors) if errors else None,
            "rms_joint_error_rad": (
                math.sqrt(sum(value * value for value in errors) / len(errors))
                if errors else None
            ),
            "motion_start_skew_s": start_skew_s,
        }
        for sample in self.hardware_samples:
            sample["clock_skew_s"] = start_skew_s or 0.0
        self.manifest.update({
            "status": "complete" if success else "incomplete",
            "reason": reason,
            "metrics": metrics,
        })
        self._write_manifest()
        calibration = self.bundle.manifest.get("calibration")
        if not isinstance(calibration, dict):
            calibration = {
                "physical_drone_radius_m": 0.067,
                "optitrack_position_error_p999_m": 0.0,
                "drone_tracking_error_p999_m": 0.0,
                "arm_tracking_cartesian_error_p999_m": 0.0,
                "latency_p999_s": 0.0,
                "maximum_relative_speed_mps": 0.0,
                "additional_safety_margin_m": 0.02,
            }
        planning = self.bundle.manifest.get("planning")
        if not isinstance(planning, dict):
            planning = {}
        hardware_log = {
            "schema_version": 1,
            "session_id": self.manifest["run_id"],
            "scene_path": self.bundle.manifest.get("scene_path", ""),
            "planned_result_path": self.bundle.manifest.get("result_path", ""),
            "expected_drone_names": [
                self.name_mapping.get(name, name)
                for name in self.bundle.robot_names
            ],
            "calibration": calibration,
            "planning": {
                "arrival_time_s": self.bundle.main.duration_s,
                "planning_time_s": float(planning.get("planning_time_s", 0.0)),
                "solve_status": bool(planning.get("solve_status", True)),
                "validation_status": True,
                "recycled_goal_roots": int(
                    planning.get("recycled_goal_roots", 0)
                ),
            },
            "execution": {
                "safety_abort": not success,
                "samples": self.hardware_samples,
            },
            "limits": {
                "maximum_clock_skew_s": 0.05,
                "maximum_ur5e_joint_error_rad": 0.087,
                "maximum_drone_position_error_m": 0.08,
                "maximum_tracking_gap_s": 0.05,
            },
            "observed": metrics,
        }
        hardware_path = self.root / "hardware_log.json"
        hardware_path.write_text(
            json.dumps(hardware_log, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        scene_path = hardware_log["scene_path"]
        if scene_path:
            audit_command = [
                sys.executable,
                "/home/duo/pRRTC/python_scripts/hardware_ur5e_crazyflie_audit.py",
                str(hardware_path),
                "--output", str(self.root / "hardware_audit.json"),
                "--paper-generated-dir", str(self.root / "paper_generated"),
            ]
            audit = subprocess.run(audit_command, capture_output=True, text=True)
            self.event("hardware_audit", {
                "returncode": audit.returncode,
                "stdout": audit.stdout,
                "stderr": audit.stderr,
            })
        self.arm_file.close()
        self.event_file.close()


class MissionNode:
    def __init__(
        self,
        mission_id: str,
        logger: CombinedLog,
        transform: WorldBaseTransform,
        robot_names: tuple[str, ...],
    ) -> None:
        import rclpy
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node("crazyfly_prrtc_coordinator")
        self.logger = logger
        self.mission_id = mission_id
        self.ready = threading.Event()
        self.scheduled = threading.Event()
        self.started = threading.Event()
        self.endpoint = threading.Event()
        self.complete = threading.Event()
        self.failed = threading.Event()
        self.drone_started_at: float | None = None
        self.transform = transform
        self.robot_names = set(robot_names)
        self.latest_positions: dict[str, tuple[float, float, float]] = {}
        self.schedule_publisher = self.node.create_publisher(
            String, "/crazyfly/mission/schedule_requests", 10
        )
        self.return_client = self.node.create_client(
            Trigger, "/crazyfly/mission/return"
        )
        self.abort_client = self.node.create_client(
            Trigger, "/crazyfly/mission/abort"
        )
        self.node.create_subscription(
            String, "/crazyfly/commands", self._event_callback, 10
        )
        from motion_capture_tracking_interfaces.msg import NamedPoseArray
        from rclpy.qos import qos_profile_sensor_data
        self.node.create_subscription(
            NamedPoseArray, "/poses", self._pose_callback, qos_profile_sensor_data
        )
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self) -> None:
        from rclpy.executors import ExternalShutdownException
        try:
            self.rclpy.spin(self.node)
        except ExternalShutdownException:
            pass

    def _event_callback(self, message) -> None:
        event = json.loads(message.data)
        self.logger.event("crazyfly_command", event)
        command = event.get("command")
        status = event.get("status")
        if command == "mission_ready" and status == "accepted":
            self.ready.set()
        elif command == "mission_schedule":
            (self.scheduled if status == "accepted" else self.failed).set()
        elif command == "trajectory_start" and status == "accepted":
            self.drone_started_at = float(event.get("monotonic_s"))
            self.started.set()
        elif command == "mission_endpoint_ready" and status == "accepted":
            self.endpoint.set()
        elif command == "mission_complete" and status == "accepted":
            self.complete.set()
        elif command in {"mission_incomplete", "emergency", "safety_land"}:
            self.failed.set()

    def _pose_callback(self, message) -> None:
        for named_pose in message.poses:
            if named_pose.name not in self.robot_names:
                continue
            point = named_pose.pose.position
            converted = self.transform.world_to_base((point.x, point.y, point.z))
            self.latest_positions[named_pose.name] = tuple(
                float(value) for value in converted
            )

    def positions(self) -> dict[str, tuple[float, float, float]]:
        return dict(self.latest_positions)

    def schedule(self, start_ns: int) -> None:
        from std_msgs.msg import String
        message = String()
        message.data = json.dumps({
            "mission_id": self.mission_id,
            "start_monotonic_ns": start_ns,
        }, separators=(",", ":"))
        self.schedule_publisher.publish(message)

    def trigger(self, client, timeout_s: float = 3.0) -> bool:
        from std_srvs.srv import Trigger
        if not client.wait_for_service(timeout_sec=timeout_s):
            return False
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return bool(future.done() and future.result().success)

    def close(self) -> None:
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()
        self.thread.join(timeout=2.0)


class MockReceive:
    def __init__(self) -> None:
        self.q = [0.0] * 6

    def getActualQ(self):
        return list(self.q)

    def getActualTCPPose(self):
        return [0.0] * 6

    def getRobotMode(self):
        return 7

    def getSafetyMode(self):
        return 1

    def disconnect(self):
        pass


class MockControl:
    def __init__(self, receive: MockReceive) -> None:
        self.receive = receive

    def moveJ(self, q, _speed, _acceleration):
        self.receive.q = list(q)
        return True

    def servoJ(self, q, *_args):
        self.receive.q = list(q)
        return True

    def initPeriod(self):
        return time.monotonic()

    def waitPeriod(self, started):
        time.sleep(max(0.0, 0.01 - (time.monotonic() - started)))

    def servoStop(self, _acceleration=10.0):
        return True

    def stopJ(self, _acceleration=2.0, _asynchronous=False):
        pass

    def disconnect(self):
        pass


def planner_joint_positions(
    physical_joints_rad: tuple[float, ...],
    first_joint_offset_rad: float,
) -> tuple[float, ...]:
    """Undo the physical base-mount offset for planner-frame geometry."""
    if len(physical_joints_rad) != 6:
        raise ConfigError("UR5e clearance model requires six joints")
    return (
        physical_joints_rad[0] - first_joint_offset_rad,
        *physical_joints_rad[1:],
    )


class RobotDistanceModel:
    def __init__(
        self,
        bundle: ExecutionBundle,
        *,
        required: bool,
        first_joint_offset_rad: float = 0.0,
    ) -> None:
        self.client_id = None
        self.robot_id = None
        self.joints: list[int] = []
        self.bodies: dict[str, int] = {}
        self.rotation = None
        self.translation = None
        self.first_joint_offset_rad = first_joint_offset_rad
        scene_path = bundle.manifest.get("scene_path")
        if not isinstance(scene_path, str):
            if required:
                raise ConfigError("physical bundle requires scene_path for clearance")
            return
        scene_file = Path(scene_path)
        if not scene_file.is_absolute():
            scene_file = Path("/home/duo/pRRTC") / scene_file
        scene = json.loads(scene_file.read_text(encoding="utf-8"))
        urdf_path = Path(str(scene["urdf"]))
        if not urdf_path.is_absolute():
            urdf_path = Path("/home/duo/pRRTC") / urdf_path
        import pybullet
        from scipy.spatial.transform import Rotation
        self.pybullet = pybullet
        frame = bundle.manifest.get("frame", {})
        transform = frame.get("payload_to_planner", {}) if isinstance(frame, dict) else {}
        quaternion = transform.get("orientation_xyzw") if isinstance(transform, dict) else None
        translation = transform.get("translation_m") if isinstance(transform, dict) else None
        if not isinstance(quaternion, list) or len(quaternion) != 4:
            raise ConfigError("physical bundle requires payload_to_planner rotation")
        if not isinstance(translation, list) or len(translation) != 3:
            raise ConfigError("physical bundle requires payload_to_planner translation")
        self.rotation = Rotation.from_quat(quaternion)
        self.translation = tuple(float(value) for value in translation)
        self.client_id = pybullet.connect(pybullet.DIRECT)
        self.robot_id = pybullet.loadURDF(
            str(urdf_path), useFixedBase=True, physicsClientId=self.client_id
        )
        self.joints = [
            index for index in range(pybullet.getNumJoints(self.robot_id))
            if pybullet.getJointInfo(self.robot_id, index)[2]
            in (pybullet.JOINT_REVOLUTE, pybullet.JOINT_PRISMATIC)
        ]
        if len(self.joints) != 6:
            raise ConfigError("URDF must expose exactly six active UR5e joints")

    def separations(
        self,
        joints_rad: tuple[float, ...],
        positions: dict[str, tuple[float, float, float]],
    ) -> dict[str, float]:
        if self.robot_id is None:
            return {name: 10.0 for name in positions}
        p = self.pybullet
        planner_joints = planner_joint_positions(
            joints_rad, self.first_joint_offset_rad
        )
        for joint, value in zip(self.joints, planner_joints):
            p.resetJointState(self.robot_id, joint, value)
        result = {}
        for name, position in positions.items():
            planner_position = self.rotation.apply(position) + self.translation
            if name not in self.bodies:
                shape = p.createCollisionShape(
                    p.GEOM_SPHERE,
                    radius=0.001,
                    physicsClientId=self.client_id,
                )
                self.bodies[name] = p.createMultiBody(
                    baseMass=0.0,
                    baseCollisionShapeIndex=shape,
                    basePosition=planner_position,
                    physicsClientId=self.client_id,
                )
            body = self.bodies[name]
            p.resetBasePositionAndOrientation(
                body,
                planner_position,
                [0.0, 0.0, 0.0, 1.0],
                physicsClientId=self.client_id,
            )
            points = p.getClosestPoints(
                self.robot_id,
                body,
                distance=5.0,
                physicsClientId=self.client_id,
            )
            result[name] = min(float(point[8]) for point in points) + 0.001
        return result

    def close(self) -> None:
        if self.client_id is not None:
            self.pybullet.disconnect(self.client_id)


def stop_process(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    with suppress(ProcessLookupError):
        os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=8.0)
    except subprocess.TimeoutExpired:
        with suppress(ProcessLookupError):
            os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=3.0)


def wait_for_mission(
    event: threading.Event,
    mission: MissionNode,
    process: subprocess.Popen,
    timeout_s: float,
    label: str,
) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if event.wait(timeout=0.05):
            return
        if mission.failed.is_set():
            raise RuntimeError(f"Crazyflie mission failed while waiting for {label}")
        code = process.poll()
        if code is not None:
            raise RuntimeError(
                f"Crazyflie mission exited with code {code} while waiting for {label}"
            )
    raise RuntimeError(f"Crazyflie mission timed out waiting for {label}")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--robot-ip", default="172.16.90.197")
    parser.add_argument("--confirm-robot-ip")
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT))
    parser.add_argument(
        "--maximum-joint-speed-rad-s", type=float, default=pi
    )
    parser.add_argument(
        "--maximum-joint-acceleration-rad-s2", type=float, default=40.0
    )
    parser.add_argument("--servo-gain", type=float, default=1000.0)
    parser.add_argument("--first-joint-offset-rad", type=float, default=pi / 2.0)
    playback = parser.add_mutually_exclusive_group()
    playback.add_argument(
        "--playback-timescale",
        type=float,
        help="duration multiplier; 2.0 runs both systems at half speed",
    )
    playback.add_argument(
        "--maximum-arm-speed-rad-s",
        type=float,
        help="derive a shared slowdown that caps every arm joint speed",
    )
    return parser


def resolve_playback_timescale(
    bundle: ExecutionBundle,
    *,
    playback_timescale: float | None,
    maximum_arm_speed_rad_s: float | None,
) -> float:
    """Resolve one shared duration multiplier for the arm and drones."""
    if playback_timescale is not None:
        if not math.isfinite(playback_timescale) or playback_timescale < 1.0:
            raise ConfigError("playback timescale must be finite and at least 1.0")
        return playback_timescale
    if maximum_arm_speed_rad_s is None:
        return 1.0
    if (
        not math.isfinite(maximum_arm_speed_rad_s)
        or maximum_arm_speed_rad_s <= 0
    ):
        raise ConfigError("maximum arm speed must be positive and finite")
    source_maximum = max(
        maximum_joint_speed(bundle.main),
        maximum_joint_speed(bundle.park),
        *(
            [maximum_joint_speed(bundle.preposition)]
            if bundle.preposition is not None
            else []
        ),
    )
    return max(1.0, source_maximum / maximum_arm_speed_rad_s)


def _print_bundle(bundle: ExecutionBundle, timescale: float = 1.0) -> None:
    print(f"PASS: pRRTC bundle {bundle.bundle_id!r} is valid")
    print(f"  drones:       {', '.join(bundle.robot_names)}")
    if bundle.preposition is not None:
        print(f"  preposition:  {bundle.preposition.duration_s:.3f} s")
    print(f"  arm duration: {bundle.main.duration_s:.3f} s")
    print(f"  park duration:{bundle.park.duration_s:.3f} s")
    print(
        "  arm return:   "
        + (
            "during uninterrupted drone motion"
            if bundle.return_during_drone_motion
            else "after drone endpoint"
        )
    )
    print(f"  playback scale:{timescale:.6f}x duration")


def complete_arm_mission_and_wait_for_drones(
    *,
    executor: TimedURExecutor,
    bundle: ExecutionBundle,
    mission_node: MissionNode,
    start_at: float,
    playback_timescale: float,
    record_sample,
    log: CombinedLog,
) -> None:
    """Return at the validated global time, then hold until drone completion."""
    hold_target = bundle.main.end
    if bundle.return_during_drone_motion:
        return_start = start_at + bundle.main.duration_s
        log.event(
            "arm_return_start",
            {
                "scheduled_monotonic_s": return_start,
                "duration_s": bundle.park.duration_s,
            },
        )
        executor.execute(
            bundle.park,
            return_start,
            record_sample,
            stop_when=mission_node.failed.is_set,
        )
        hold_target = bundle.park.end
        log.event("arm_home_reached", {"monotonic_s": time.monotonic()})

    hold_deadline = (
        start_at
        + float(bundle.drone_payload["duration_s"]) * playback_timescale
        + 3.0
    )
    if not executor.hold_until(
        hold_target,
        hold_deadline,
        lambda: mission_node.endpoint.is_set() or mission_node.failed.is_set(),
        on_sample=record_sample,
    ) or mission_node.failed.is_set():
        raise RuntimeError("Crazyflie endpoint acknowledgement timed out")

    if not bundle.return_during_drone_motion:
        executor.execute(
            bundle.park,
            time.monotonic() + 0.02,
            record_sample,
            stop_when=mission_node.failed.is_set,
        )


def run_combined(args, bundle: ExecutionBundle) -> Path:
    log = CombinedLog(Path(args.output_root), bundle)
    log.manifest["execution_parameters"] = {
        "maximum_joint_speed_rad_s": args.maximum_joint_speed_rad_s,
        "maximum_joint_acceleration_rad_s2": (
            args.maximum_joint_acceleration_rad_s2
        ),
        "servo_gain": args.servo_gain,
        "servo_lookahead_s": URExecutionConfig().servo_lookahead_s,
        "first_joint_offset_rad": args.first_joint_offset_rad,
        "playback_timescale": args.playback_timescale,
        "accepted_missing_physical_evidence": True,
        "return_altitudes_m": [0.2, 0.4, 0.6, 0.8, 1.0],
        "home_first_preposition": bundle.preposition is not None,
        "return_during_drone_motion": bundle.return_during_drone_motion,
        "preposition_duration_s": (
            None
            if bundle.preposition is None
            else bundle.preposition.duration_s
        ),
    }
    log._write_manifest()
    process = None
    mission_node = None
    executor = None
    distance_model = None
    start_skew = None
    success = False
    reason = ""
    try:
        mission_id = str(bundle.drone_payload["mission_id"])
        transform = WorldBaseTransform.load(DEFAULT_CALIBRATION)
        mission_node = MissionNode(
            mission_id, log, transform, bundle.robot_names
        )
        distance_model = RobotDistanceModel(
            bundle,
            required=not args.mock,
            first_joint_offset_rad=args.first_joint_offset_rad,
        )
        safety = load_safety(DEFAULT_SAFETY)
        _, _, drone_plan = trajectory_from_payload(
            bundle.drone_payload, bundle.robot_names, safety
        )
        command = [
            sys.executable,
            str(ROOT / "tools" / "payload_mission.py"),
            str(bundle.root / "crazyfly_trajectory_payload.json"),
            "--robots", ",".join(bundle.robot_names),
            "--execute", "--wait-for-start", "--wait-for-return",
            "--start-timeout-s", "120",
            "--return-timeout-s", str(max(10.0, bundle.park.duration_s + 5.0)),
            "--playback-timescale", str(args.playback_timescale),
            "--output-root", str(log.root / "crazyfly"),
        ]
        if args.mock:
            command.append("--mock")
        if args.mock:
            receive = MockReceive()
            control = MockControl(receive)
        else:
            control, receive = connect_ur5e(args.robot_ip)
        executor = TimedURExecutor(
            control,
            receive,
            URExecutionConfig(servo_gain=args.servo_gain),
        )
        executor.validate_robot_ready()
        executor.preposition(
            bundle.main.start
            if bundle.preposition is None
            else bundle.preposition.start
        )
        process = subprocess.Popen(command, start_new_session=True)
        wait_for_mission(mission_node.ready, mission_node, process, 90.0, "ready")

        if bundle.preposition is not None:
            log.event(
                "arm_preposition_start",
                {"duration_s": bundle.preposition.duration_s},
            )

            def record_preposition_sample(sample: URSample) -> None:
                log.arm_sample(sample)
                measured = mission_node.positions()
                separations = distance_model.separations(
                    sample.actual_rad, measured
                )
                log.hardware_sample(
                    sample,
                    drone_plan.start_positions,
                    measured,
                    separations,
                )

            executor.execute(
                bundle.preposition,
                time.monotonic() + 0.02,
                record_preposition_sample,
                stop_when=mission_node.failed.is_set,
            )
            log.event("arm_preposition_complete", {})

        start_ns = time.monotonic_ns() + 2_000_000_000
        mission_node.schedule(start_ns)
        wait_for_mission(
            mission_node.scheduled, mission_node, process, 3.0, "schedule"
        )
        start_at = start_ns / 1_000_000_000.0

        def record_sample(sample: URSample) -> None:
            log.arm_sample(sample)
            drone_elapsed = min(
                max(0.0, sample.monotonic_s - start_at)
                / args.playback_timescale,
                drone_plan.duration_s,
            )
            commanded = evaluate_plan(drone_plan, drone_elapsed)
            measured = mission_node.positions()
            separations = distance_model.separations(sample.actual_rad, measured)
            log.hardware_sample(sample, commanded, measured, separations)

        main_samples = executor.execute(
            bundle.main,
            start_at,
            on_sample=record_sample,
            stop_when=mission_node.failed.is_set,
        )
        wait_for_mission(
            mission_node.started, mission_node, process, 2.0, "trajectory start"
        )
        start_skew = abs(main_samples[0].monotonic_s - mission_node.drone_started_at)
        if start_skew > 0.05:
            raise RuntimeError(f"motion start skew exceeded 50 ms: {start_skew:.3f}")
        complete_arm_mission_and_wait_for_drones(
            executor=executor,
            bundle=bundle,
            mission_node=mission_node,
            start_at=start_at,
            playback_timescale=args.playback_timescale,
            record_sample=record_sample,
            log=log,
        )
        if not mission_node.trigger(mission_node.return_client):
            raise RuntimeError("Crazyflie return release failed")
        wait_for_mission(
            mission_node.complete, mission_node, process, 45.0, "completion"
        )
        executor.stop()
        success = True
        return log.root
    except BaseException as exc:
        reason = str(exc)
        if executor is not None:
            with suppress(Exception):
                executor.stop()
        if mission_node is not None:
            with suppress(Exception):
                mission_node.trigger(mission_node.abort_client)
        raise
    finally:
        if executor is not None:
            with suppress(Exception):
                executor.disconnect()
        if distance_model is not None:
            with suppress(Exception):
                distance_model.close()
        if mission_node is not None:
            mission_node.close()
        stop_process(process)
        log.finish(success=success, start_skew_s=start_skew, reason=reason)


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if (
            not math.isfinite(args.maximum_joint_speed_rad_s)
            or args.maximum_joint_speed_rad_s <= 0
        ):
            raise ConfigError("maximum joint speed must be positive and finite")
        if (
            not math.isfinite(args.maximum_joint_acceleration_rad_s2)
            or args.maximum_joint_acceleration_rad_s2 <= 0
        ):
            raise ConfigError(
                "maximum joint acceleration must be positive and finite"
            )
        if not math.isfinite(args.servo_gain) or not 100 <= args.servo_gain <= 2000:
            raise ConfigError("servo gain must be from 100 to 2000")
        if not math.isfinite(args.first_joint_offset_rad):
            raise ConfigError("first joint offset must be finite")
        bundle = load_execution_bundle(
            args.bundle,
            require_clean=False,
            maximum_joint_speed_rad_s=args.maximum_joint_speed_rad_s,
            maximum_joint_acceleration_rad_s2=(
                args.maximum_joint_acceleration_rad_s2
            ),
            accept_missing_physical_evidence=True,
            first_joint_offset_rad=args.first_joint_offset_rad,
        )
        args.playback_timescale = resolve_playback_timescale(
            bundle,
            playback_timescale=args.playback_timescale,
            maximum_arm_speed_rad_s=args.maximum_arm_speed_rad_s,
        )
        safety = load_safety(DEFAULT_SAFETY)
        scaled_drone_duration = (
            float(bundle.drone_payload["duration_s"])
            * args.playback_timescale
        )
        if scaled_drone_duration > safety.maximum_trajectory_duration_s:
            raise ConfigError(
                "scaled drone duration exceeds the configured mission limit"
            )
        bundle = replace(
            bundle,
            preposition=(
                None
                if bundle.preposition is None
                else scale_trajectory_time(
                    bundle.preposition, args.playback_timescale
                )
            ),
            main=scale_trajectory_time(bundle.main, args.playback_timescale),
            park=scale_trajectory_time(bundle.park, args.playback_timescale),
        )
        WorldBaseTransform.load(DEFAULT_CALIBRATION)
        _print_bundle(bundle, args.playback_timescale)
        if not args.execute:
            print("DRY RUN: no ROS process, RTDE connection, or command was started")
            return 0
        if not args.mock:
            if args.confirm_robot_ip != args.robot_ip:
                raise ConfigError("--confirm-robot-ip must match the UR5e robot IP")
            validate_physical_metadata(
                bundle,
                accept_missing_evidence=True,
            )
            states = (
                git_state(ROOT),
                git_state(UR_TOOLS_ROOT),
                git_state(PRRTC_ROOT),
            )
            if any(state["dirty"] for state in states):
                raise ConfigError(
                    "physical execution requires clean crazyfly, ur_tools, "
                    "and current pRRTC repositories"
                )
        output = run_combined(args, bundle)
        print(f"COMBINED MISSION COMPLETE: {output}")
        return 0
    except KeyboardInterrupt:
        print("Interrupted; combined cleanup requested", file=sys.stderr)
        return 130
    except (ConfigError, RuntimeError, OSError, subprocess.SubprocessError) as exc:
        print(f"FAILED: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
