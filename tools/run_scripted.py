"""Run a deterministic baseline controller and write a trajectory exchange file."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from racing_event.config import SimulationConfig
from racing_event.env import RacingEnv
from racing_event.track import TrackGeometry


def environment_hash(track: TrackGeometry, config_path: Path) -> str:
    digest = hashlib.sha256()
    digest.update(track.content_hash.encode("ascii"))
    digest.update(config_path.read_bytes())
    return digest.hexdigest()


def scripted_action(env: RacingEnv) -> np.ndarray:
    """A deliberately small controller used only for simulator smoke tests."""
    def scripted_action(env: RacingEnv) -> np.ndarray:
    """Corner-aware baseline controller with early braking."""

    
def scripted_action(env: RacingEnv) -> np.ndarray:
    """Corner-aware baseline controller with early braking."""

    projection = env.track.project(
        np.array([env.state.x, env.state.y], dtype=np.float64)
    )

    speed = env.state.speed
    car = env.config.car

    lookahead = max(15.0, speed * 0.75)
    curvature = env.track.curvature_at_s(projection.s + lookahead)

    desired_angle = np.arctan(car.wheelbase_m * curvature)
    steering = desired_angle / car.max_steering_rad
    steering -= 0.9 * np.clip(projection.lateral_offset / 8.0, -1.0, 1.0)
    steering -= 0.8 * env.track.observation_heading_error(
        env.state.heading, projection
    )

    target_speed = car.max_speed_mps

    for distance in [15.0, 40.0, 80.0]:
        future_curvature = env.track.curvature_at_s(projection.s + distance)
        corner_speed = (
            car.peak_lateral_accel_mps2
            / max(abs(future_curvature), 1e-4)
        ) ** 0.5

        corner_speed *= 0.85
        allowed_speed = (corner_speed ** 2 + 2.0 * 5.0 * distance) ** 0.5
        target_speed = min(target_speed, allowed_speed)

    longitudinal = np.clip((target_speed - speed) * 0.25, -1.0, 0.65)

    return np.clip(
        np.asarray([steering, longitudinal], dtype=np.float64),
        -1.0,
        1.0
    )


    # Steer using curvature ahead, not just the current track position.
    lookahead = max(15.0, speed * 0.75)
    curvature = env.track.curvature_at_s(projection.s + lookahead)

    desired_angle = np.arctan(car.wheelbase_m * curvature)
    steering = desired_angle / car.max_steering_rad
    steering -= 0.9 * np.clip(projection.lateral_offset / 8.0, -1.0, 1.0)
    steering -= 0.8 * env.track.observation_heading_error(
        env.state.heading, projection
    )

    # Estimate safe speeds at several distances ahead.
    target_speed = car.max_speed_mps

    for distance in [15.0, 40.0, 80.0]:
        future_curvature = env.track.curvature_at_s(projection.s + distance)

        corner_speed = (
            car.peak_lateral_accel_mps2
            / max(abs(future_curvature), 1e-4)
        ) ** 0.5

        # Leave a safety margin for the corner.
        corner_speed *= 0.85

        # Brake early enough to reach that speed before the corner.
        allowed_speed = (
            corner_speed ** 2 + 2.0 * 5.0 * distance
        ) ** 0.5

        target_speed = min(target_speed, allowed_speed)

    # Positive = throttle; negative = braking.
    longitudinal = np.clip(
        (target_speed - speed) * 0.25,
        -1.0,
        0.65
    )

    return np.clip(
        np.asarray([steering, longitudinal], dtype=np.float64),
        -1.0,
        1.0
    )




def run(track_path: Path, config_path: Path, steps: int, participant_id: str) -> dict[str, object]:
    track = TrackGeometry.from_file(track_path)
    config = SimulationConfig.from_file(config_path)
    env = RacingEnv(track, config)
    env.reset(seed=0, spawn_s=track.start_line_s, lateral_offset=0.0, initial_speed=0.0, heading_error=0.0)
    frames: list[dict[str, object]] = []
    status = "dnf_timeout"
    lap_time_s: float | None = None
    for tick in range(steps):
        action = scripted_action(env)
        _, _, terminated, truncated, info = env.step(action)
        state = info["state"]
        assert isinstance(state, dict)
        frames.append(
            {
                "tick": tick,
                "x": state["x"],
                "y": state["y"],
                "heading": state["heading"],
                "speed": state["speed"],
                "steer": state["steer"],
                "throttle_brake": float(action[1]),
                "yaw_rate": state["yaw_rate"],
                "off_track": bool(info["off_track"]),
                "events": list(info["events"]),
            }
        )
        if terminated or truncated:
            if info["lap_complete"]:
                status = "finished"
                lap_time_s = (tick + 1) * config.dt
            elif info["crash"]:
                status = "dnf_crash"
            elif env.stuck_ticks >= config.episode.stuck_steps:
                status = "dnf_stuck"
            break
    return {
        "schema_version": "1.0",
        "env_hash": environment_hash(track, config_path),
        "track_id": track.track_id,
        "track_hash": track.content_hash,
        "tick_hz": config.tick_hz,
        "participant_id": participant_id,
        "result": {"status": status, "lap_time_s": lap_time_s},
        "frames": frames,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--track", type=Path, default=Path("tracks/train/monaco.json"))
    parser.add_argument("--config", type=Path, default=Path("config/simulation.json"))
    parser.add_argument("--steps", type=int, default=120)
    parser.add_argument("--participant-id", default="scripted_smoke_test")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.steps <= 0:
        parser.error("--steps must be positive")
    result = run(args.track, args.config, args.steps, args.participant_id)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"wrote {args.output} ({len(result['frames'])} frames, {result['result']['status']})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
