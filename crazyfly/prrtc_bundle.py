"""Validation and interpolation for pRRTC hardware execution bundles."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
from math import isfinite, pi
from pathlib import Path
from typing import Mapping, Sequence

from .config import ConfigError


JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
REQUIRED_FILES = (
    "crazyfly_trajectory_payload.json",
    "ur5e_trajectory.json",
    "ur5e_park_trajectory.json",
    "validation.json",
)
MAXIMUM_JOINT_SPEED_RAD_S = pi
MAXIMUM_JOINT_ACCELERATION_RAD_S2 = 40.0
MAXIMUM_PREPOSITION_DURATION_S = 15.5
MAXIMUM_PREPOSITION_SPEED_RAD_S = 0.5
MAXIMUM_PREPOSITION_ACCELERATION_RAD_S2 = 1.0


@dataclass(frozen=True)
class JointSample:
    time_s: float
    positions_rad: tuple[float, float, float, float, float, float]


@dataclass(frozen=True)
class JointTrajectory:
    name: str
    samples: tuple[JointSample, ...]

    @property
    def duration_s(self) -> float:
        return self.samples[-1].time_s

    @property
    def start(self) -> tuple[float, ...]:
        return self.samples[0].positions_rad

    @property
    def end(self) -> tuple[float, ...]:
        return self.samples[-1].positions_rad

    def evaluate(self, time_s: float) -> tuple[float, ...]:
        if time_s <= 0:
            return self.start
        if time_s >= self.duration_s:
            return self.end
        for first, second in zip(self.samples, self.samples[1:]):
            if time_s <= second.time_s:
                fraction = (time_s - first.time_s) / (
                    second.time_s - first.time_s
                )
                return tuple(
                    start + fraction * (end - start)
                    for start, end in zip(
                        first.positions_rad, second.positions_rad
                    )
                )
        return self.end


@dataclass(frozen=True)
class ExecutionBundle:
    root: Path
    bundle_id: str
    drone_payload: dict[str, object]
    main: JointTrajectory
    park: JointTrajectory
    validation: dict[str, object]
    manifest: dict[str, object]
    robot_names: tuple[str, ...]
    preposition: JointTrajectory | None = None
    return_during_drone_motion: bool = False


def validate_physical_metadata(
    bundle: ExecutionBundle,
    *,
    accept_missing_evidence: bool = False,
) -> None:
    mapping = bundle.manifest.get("name_mapping")
    if not isinstance(mapping, Mapping) or set(mapping) != set(bundle.robot_names):
        raise ConfigError("physical bundle requires exact hardware name_mapping")
    for field in ("scene_path", "result_path"):
        if not isinstance(bundle.manifest.get(field), str) or not bundle.manifest[field]:
            raise ConfigError(f"physical bundle requires {field}")
    if accept_missing_evidence:
        return
    calibration = bundle.manifest.get("calibration")
    required = (
        "physical_drone_radius_m",
        "optitrack_position_error_p999_m",
        "drone_tracking_error_p999_m",
        "arm_tracking_cartesian_error_p999_m",
        "latency_p999_s",
        "maximum_relative_speed_mps",
        "additional_safety_margin_m",
    )
    if not isinstance(calibration, Mapping) or any(
        _finite(calibration.get(field), f"calibration {field}") <= 0
        for field in required
    ):
        raise ConfigError("physical bundle requires positive measured calibration fields")


def _read_object(path: Path) -> dict[str, object]:
    try:
        value: object = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ConfigError(f"invalid bundle file {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ConfigError(f"bundle file must contain an object: {path}")
    return value


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _finite(value: object, field: str) -> float:
    if isinstance(value, bool):
        raise ConfigError(f"{field} must be finite")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"{field} must be finite") from exc
    if not isfinite(result):
        raise ConfigError(f"{field} must be finite")
    return result


def load_joint_trajectory(
    path: Path,
    name: str,
    *,
    allow_missing_frame: bool = False,
    normalize_time_origin: bool = False,
    allow_schema_version_2: bool = False,
    allow_schema_version_3: bool = False,
) -> JointTrajectory:
    payload = _read_object(path)
    frame = payload.get("frame")
    if frame != "base" and not (allow_missing_frame and frame is None):
        raise ConfigError(f"{name} must use the physical base frame")
    if tuple(payload.get("joint_names", ())) != JOINT_NAMES:
        raise ConfigError(f"{name} has unexpected UR5e joint names")
    raw_samples = payload.get("samples")
    if not isinstance(raw_samples, list) or len(raw_samples) < 2:
        raise ConfigError(f"{name} requires at least two joint samples")
    samples = []
    previous = -1.0
    time_origin: float | None = None
    for index, raw in enumerate(raw_samples):
        if not isinstance(raw, Mapping):
            raise ConfigError(f"{name} sample {index} must be an object")
        raw_time_s = _finite(raw.get("time_s"), f"{name} sample time")
        if time_origin is None:
            time_origin = raw_time_s if normalize_time_origin else 0.0
        time_s = raw_time_s - time_origin
        positions = raw.get("positions_rad")
        if not isinstance(positions, Sequence) or isinstance(positions, (str, bytes)):
            raise ConfigError(f"{name} sample {index} requires six joints")
        values = tuple(_finite(value, f"{name} joint") for value in positions)
        if len(values) != 6:
            raise ConfigError(f"{name} sample {index} requires six joints")
        if index == 0 and abs(time_s) > 1e-7:
            raise ConfigError(f"{name} must begin at zero")
        if time_s <= previous:
            raise ConfigError(f"{name} times must be strictly increasing")
        previous = time_s
        samples.append(JointSample(time_s, values))  # type: ignore[arg-type]
    return JointTrajectory(name=name, samples=tuple(samples))


def validate_joint_limits(
    trajectory: JointTrajectory,
    maximum_speed_rad_s: float = MAXIMUM_JOINT_SPEED_RAD_S,
    maximum_acceleration_rad_s2: float = MAXIMUM_JOINT_ACCELERATION_RAD_S2,
) -> None:
    velocities: list[tuple[float, ...]] = []
    durations: list[float] = []
    for first, second in zip(trajectory.samples, trajectory.samples[1:]):
        duration = second.time_s - first.time_s
        durations.append(duration)
        velocity = tuple(
            (end - start) / duration
            for start, end in zip(first.positions_rad, second.positions_rad)
        )
        velocities.append(velocity)
        if max(abs(value) for value in velocity) > maximum_speed_rad_s + 1e-7:
            raise ConfigError(f"{trajectory.name} exceeds joint speed limit")
    for index, (first, second) in enumerate(zip(velocities, velocities[1:])):
        interval = 0.5 * (durations[index] + durations[index + 1])
        acceleration = max(
            abs(after - before) / interval
            for before, after in zip(first, second)
        )
        if acceleration > maximum_acceleration_rad_s2 + 1e-7:
            raise ConfigError(f"{trajectory.name} exceeds joint acceleration limit")


def offset_first_joint(
    trajectory: JointTrajectory,
    offset_rad: float,
) -> JointTrajectory:
    """Return a physical-installation trajectory with joint 1 offset."""
    if not isfinite(offset_rad):
        raise ConfigError("first joint offset must be finite")
    return JointTrajectory(
        name=trajectory.name,
        samples=tuple(
            JointSample(
                time_s=sample.time_s,
                positions_rad=(
                    sample.positions_rad[0] + offset_rad,
                    *sample.positions_rad[1:],
                ),
            )
            for sample in trajectory.samples
        ),
    )


def maximum_joint_speed(trajectory: JointTrajectory) -> float:
    """Return the maximum absolute piecewise-linear joint speed."""
    return max(
        abs(end - start) / (second.time_s - first.time_s)
        for first, second in zip(trajectory.samples, trajectory.samples[1:])
        for start, end in zip(first.positions_rad, second.positions_rad)
    )


def scale_trajectory_time(
    trajectory: JointTrajectory,
    timescale: float,
) -> JointTrajectory:
    """Slow a joint trajectory by multiplying every timestamp."""
    if not isfinite(timescale) or timescale < 1.0:
        raise ConfigError("trajectory timescale must be finite and at least 1.0")
    return JointTrajectory(
        name=trajectory.name,
        samples=tuple(
            JointSample(
                time_s=sample.time_s * timescale,
                positions_rad=sample.positions_rad,
            )
            for sample in trajectory.samples
        ),
    )


def _require_collision_validation(validation: Mapping[str, object], key: str) -> None:
    item = validation.get(key)
    if not isinstance(item, Mapping) or item.get("collision_free") is not True:
        raise ConfigError(f"bundle requires collision-free {key} validation")


def _normalize_v2_bundle(
    manifest: Mapping[str, object],
    validation: Mapping[str, object],
) -> tuple[dict[str, object], dict[str, object]]:
    """Translate pRRTC's version-2 names without weakening validation."""
    normalized = deepcopy(dict(manifest))
    normalized["source_schema_version"] = manifest.get("schema_version")
    normalized["files"] = manifest.get("artifact_hashes")
    durations = manifest.get("durations_s")
    if isinstance(durations, Mapping):
        normalized["T_goal_s"] = durations.get("main_arm")
    frames = manifest.get("frames")
    if isinstance(frames, Mapping):
        normalized["frame"] = {
            "payload_to_planner": frames.get("payload_to_planner"),
        }
    sources = manifest.get("sources")
    if isinstance(sources, Mapping):
        scene = sources.get("planner_scene")
        result = sources.get("planner_result")
        if isinstance(scene, Mapping):
            normalized["scene_path"] = scene.get("repository_path")
        if isinstance(result, Mapping):
            normalized["result_path"] = result.get("path")
    raw_mapping = manifest.get("name_mapping")
    if isinstance(raw_mapping, Mapping):
        normalized["name_mapping"] = {
            str(hardware_id): str(description)
            for description, hardware_id in raw_mapping.items()
        }
    provenance = manifest.get("pRRTC")
    if isinstance(provenance, Mapping):
        planner = provenance.get("planner")
        if isinstance(planner, Mapping):
            normalized["pRRTC_source"] = provenance
            normalized["pRRTC"] = dict(planner)

    normalized_validation = deepcopy(dict(validation))
    normalized_validation["post_goal"] = {
        "goal_hold": (
            validation.get("goal_hold_through_drone_brake")
            or validation.get("goal_hold_through_cf1_brake")
        ),
    }
    normalized_validation["arm_park"] = (
        validation.get("park_path_with_drones_held")
        or validation.get("park_path_with_cf1_held")
    )
    normalized_validation["drone_return"] = (
        validation.get("drone_return_after_park")
        or validation.get("cf1_return_after_park")
    )
    normalized_validation["abort_land_in_place"] = validation.get(
        "vertical_land_abort_corridors"
    )
    normalized_validation["arm_preposition"] = validation.get(
        "preposition_path_with_drones_held_at_start"
    )
    return normalized, normalized_validation


def _normalize_continuous_bundle(
    manifest: Mapping[str, object],
    validation: Mapping[str, object],
) -> tuple[dict[str, object], dict[str, object]]:
    """Normalize a structurally valid continuous arm-return bundle."""
    normalized = deepcopy(dict(manifest))
    normalized["source_schema_version"] = manifest.get("schema_version")
    normalized["files"] = manifest.get("artifact_hashes")
    normalized["execution_mode"] = "continuous_arm_return"

    arrival_times = manifest.get("goal_arrival_times_s")
    if (
        isinstance(arrival_times, Sequence)
        and not isinstance(arrival_times, (str, bytes))
        and arrival_times
    ):
        normalized["T_goal_s"] = arrival_times[-1]

    frames = manifest.get("frames")
    if isinstance(frames, Mapping):
        normalized["frame"] = {
            "payload_to_planner": frames.get("payload_to_planner"),
        }

    sources = manifest.get("sources")
    if isinstance(sources, Mapping):
        scene = sources.get("scene")
        if isinstance(scene, Mapping):
            normalized["scene_path"] = scene.get("snapshot_artifact")
        normalized["result_path"] = "planner_result.json"

    raw_mapping = manifest.get("name_mapping")
    if isinstance(raw_mapping, Mapping):
        normalized["name_mapping"] = {
            str(hardware_id): str(description)
            for description, hardware_id in raw_mapping.items()
        }

    provenance = manifest.get("pRRTC")
    if isinstance(provenance, Mapping):
        planner = provenance.get("planner")
        if isinstance(planner, Mapping):
            normalized["pRRTC_source"] = provenance
            normalized["pRRTC"] = dict(planner)

    normalized_validation = deepcopy(dict(validation))
    normalized_validation["arm_park"] = validation.get("return_home")
    return normalized, normalized_validation


def _inactive_preposition_state(
    payload: Mapping[str, object],
) -> tuple[float, ...] | None:
    """Validate and return an optional zero-duration home-state record."""
    if payload.get("required") is not False:
        return None
    raw_samples = payload.get("samples")
    if not isinstance(raw_samples, list) or len(raw_samples) != 1:
        raise ConfigError(
            "inactive UR5e preposition must contain exactly one home sample"
        )
    if tuple(payload.get("joint_names", ())) != JOINT_NAMES:
        raise ConfigError("inactive UR5e preposition has unexpected joint names")
    sample = raw_samples[0]
    if not isinstance(sample, Mapping):
        raise ConfigError("inactive UR5e preposition sample must be an object")
    if abs(_finite(sample.get("time_s"), "inactive preposition time")) > 1e-7:
        raise ConfigError("inactive UR5e preposition must begin at zero")
    positions = sample.get("positions_rad")
    if not isinstance(positions, Sequence) or isinstance(positions, (str, bytes)):
        raise ConfigError("inactive UR5e preposition requires six joints")
    state = tuple(_finite(value, "inactive preposition joint") for value in positions)
    if len(state) != 6:
        raise ConfigError("inactive UR5e preposition requires six joints")
    if abs(_finite(payload.get("duration_s"), "inactive preposition duration")) > 1e-7:
        raise ConfigError("inactive UR5e preposition duration must be zero")
    return state


def _validate_complete_trajectory(
    complete: JointTrajectory,
    main: JointTrajectory,
    park: JointTrajectory,
) -> None:
    expected = list(main.samples)
    expected.extend(
        JointSample(
            time_s=main.duration_s + sample.time_s,
            positions_rad=sample.positions_rad,
        )
        for sample in park.samples[1:]
    )
    if len(complete.samples) != len(expected):
        raise ConfigError(
            "authoritative complete UR5e trajectory does not match split paths"
        )
    for actual, wanted in zip(complete.samples, expected):
        if abs(actual.time_s - wanted.time_s) > 1e-7 or max(
            abs(first - second)
            for first, second in zip(actual.positions_rad, wanted.positions_rad)
        ) > 1e-7:
            raise ConfigError(
                "authoritative complete UR5e trajectory does not match split paths"
            )


def load_execution_bundle(
    root: str | Path,
    *,
    require_clean: bool = False,
    maximum_joint_speed_rad_s: float = MAXIMUM_JOINT_SPEED_RAD_S,
    maximum_joint_acceleration_rad_s2: float = (
        MAXIMUM_JOINT_ACCELERATION_RAD_S2
    ),
    accept_missing_physical_evidence: bool = False,
    first_joint_offset_rad: float = 0.0,
) -> ExecutionBundle:
    directory = Path(root).expanduser().resolve()
    raw_manifest = _read_object(directory / "manifest.json")
    raw_validation = _read_object(directory / "validation.json")

    artifact_layout = isinstance(raw_manifest.get("artifact_hashes"), Mapping)
    continuous_return = (
        raw_manifest.get("mission_mode") == "continuous_ordered_stop"
    )
    if artifact_layout and continuous_return:
        manifest, validation = _normalize_continuous_bundle(
            raw_manifest, raw_validation
        )
    elif artifact_layout:
        manifest, validation = _normalize_v2_bundle(
            raw_manifest, raw_validation
        )
    else:
        manifest = raw_manifest
        validation = raw_validation

    if artifact_layout:
        scene_snapshot = directory / "planner_scene.json"
        result_snapshot = directory / "planner_result.json"
        if scene_snapshot.is_file():
            manifest["scene_path"] = str(scene_snapshot)
        if result_snapshot.is_file():
            manifest["result_path"] = str(result_snapshot)

    bundle_id = manifest.get("bundle_id")
    if not isinstance(bundle_id, str) or not bundle_id:
        raise ConfigError("bundle manifest requires bundle_id")
    file_records = manifest.get("files")
    if not isinstance(file_records, Mapping):
        raise ConfigError("bundle manifest requires file hashes")
    required_files = list(REQUIRED_FILES)
    if continuous_return:
        required_files.append("ur5e_complete_trajectory.json")
    for filename in required_files:
        record = file_records.get(filename)
        path = directory / filename
        if not isinstance(record, Mapping) or record.get("sha256") != _sha256(path):
            raise ConfigError(f"bundle hash mismatch: {filename}")

    preposition_path = directory / "ur5e_preposition_trajectory.json"
    preposition_record = file_records.get("ur5e_preposition_trajectory.json")
    if preposition_path.is_file() != isinstance(preposition_record, Mapping):
        raise ConfigError(
            "bundle must provide both ur5e_preposition_trajectory.json and its hash"
        )
    if preposition_path.is_file() and preposition_record.get("sha256") != _sha256(
        preposition_path
    ):
        raise ConfigError("bundle hash mismatch: ur5e_preposition_trajectory.json")

    if require_clean:
        state = manifest.get("pRRTC")
        if not isinstance(state, Mapping) or state.get("dirty") is not False:
            raise ConfigError("physical execution requires a clean pRRTC bundle")

    if validation.get("offline_export_eligible") is not True:
        raise ConfigError("bundle is not offline_export_eligible")
    if continuous_return:
        if (
            manifest.get("drone_brake_applied") is not False
            or validation.get("drone_brake_applied") is not False
        ):
            raise ConfigError(
                "continuous arm-return bundle must preserve unbraked drone motion"
            )
        for key in (
            "complete_robot_mission",
            "return_home",
            "home_hold_through_drone_completion",
        ):
            _require_collision_validation(validation, key)
        boundaries = validation.get("robot_boundaries")
        if not isinstance(boundaries, Mapping) or boundaries.get("passed") is not True:
            raise ConfigError("continuous arm-return bundle requires valid boundaries")
        motion_limits = validation.get("arm_motion_limits")
        if (
            not isinstance(motion_limits, Mapping)
            or motion_limits.get("passed") is not True
        ):
            raise ConfigError("continuous arm-return bundle requires valid arm limits")
        required_validations: list[str] = []
    else:
        post_goal = validation.get("post_goal")
        if not isinstance(post_goal, Mapping):
            raise ConfigError("bundle requires post_goal validation")
        _require_collision_validation(post_goal, "goal_hold")
        required_validations = ["arm_park"]

    if not accept_missing_physical_evidence:
        required_validations.extend(("drone_return", "abort_land_in_place"))
    for key in required_validations:
        _require_collision_validation(validation, key)

    preposition_payload = (
        _read_object(preposition_path) if preposition_path.is_file() else None
    )
    inactive_preposition = (
        None
        if preposition_payload is None
        else _inactive_preposition_state(preposition_payload)
    )
    if preposition_path.is_file() and inactive_preposition is None:
        _require_collision_validation(validation, "arm_preposition")

    drone_payload = _read_object(directory / "crazyfly_trajectory_payload.json")
    if drone_payload.get("frame") != "base":
        raise ConfigError("Crazyflie payload must use physical base frame")
    raw_trajectories = drone_payload.get("trajectories")
    if not isinstance(raw_trajectories, Mapping) or not raw_trajectories:
        raise ConfigError("Crazyflie payload requires trajectories")
    robot_names = tuple(sorted(str(name) for name in raw_trajectories))

    main = load_joint_trajectory(directory / "ur5e_trajectory.json", "ur5e main")
    preposition = None
    if preposition_path.is_file() and inactive_preposition is None:
        preposition = load_joint_trajectory(
            preposition_path,
            "ur5e preposition",
            allow_missing_frame=True,
            allow_schema_version_3=True,
        )
    park = load_joint_trajectory(
        directory / "ur5e_park_trajectory.json",
        "ur5e park",
        allow_missing_frame=artifact_layout,
        normalize_time_origin=artifact_layout,
        allow_schema_version_2=artifact_layout,
        allow_schema_version_3=artifact_layout,
    )
    if continuous_return:
        complete = load_joint_trajectory(
            directory / "ur5e_complete_trajectory.json",
            "complete UR5e mission",
        )
        _validate_complete_trajectory(complete, main, park)

    validate_joint_limits(
        main,
        maximum_joint_speed_rad_s,
        maximum_joint_acceleration_rad_s2,
    )
    validate_joint_limits(
        park,
        maximum_joint_speed_rad_s,
        maximum_joint_acceleration_rad_s2,
    )
    if preposition is not None:
        validate_joint_limits(
            preposition,
            MAXIMUM_PREPOSITION_SPEED_RAD_S,
            MAXIMUM_PREPOSITION_ACCELERATION_RAD_S2,
        )
        if preposition.duration_s > MAXIMUM_PREPOSITION_DURATION_S + 1e-7:
            raise ConfigError("UR5e preposition path exceeds 15.5 second limit")
        if max(
            abs(first - second)
            for first, second in zip(preposition.end, main.start)
        ) > 1e-5:
            raise ConfigError(
                "UR5e preposition path must end at the main-path start"
            )
        if max(
            abs(first - second)
            for first, second in zip(preposition.start, park.end)
        ) > 1e-5:
            raise ConfigError(
                "UR5e preposition must start at the park-path home"
            )
    elif inactive_preposition is not None:
        if max(
            abs(first - second)
            for first, second in zip(inactive_preposition, main.start)
        ) > 1e-7 or max(
            abs(first - second)
            for first, second in zip(inactive_preposition, park.end)
        ) > 1e-7:
            raise ConfigError(
                "inactive UR5e preposition must equal mission start and home"
            )

    if max(abs(first - second) for first, second in zip(main.end, park.start)) > 1e-5:
        raise ConfigError("UR5e park path must begin at the main-path goal")
    if continuous_return:
        drone_duration = _finite(
            drone_payload.get("duration_s"), "Crazyflie payload duration"
        )
        if main.duration_s + park.duration_s > drone_duration + 1e-4:
            raise ConfigError(
                "arm must reach home before the drone trajectory ends"
            )
    manifest_goal = _finite(manifest.get("T_goal_s"), "manifest T_goal_s")
    if abs(main.duration_s - manifest_goal) > 1e-4:
        raise ConfigError("UR5e main duration does not match manifest T_goal_s")

    main = offset_first_joint(main, first_joint_offset_rad)
    park = offset_first_joint(park, first_joint_offset_rad)
    if preposition is not None:
        preposition = offset_first_joint(preposition, first_joint_offset_rad)
    manifest["physical_first_joint_offset_rad"] = first_joint_offset_rad
    return ExecutionBundle(
        root=directory,
        bundle_id=bundle_id,
        drone_payload=drone_payload,
        main=main,
        park=park,
        validation=validation,
        manifest=manifest,
        robot_names=robot_names,
        preposition=preposition,
        return_during_drone_motion=continuous_return,
    )
