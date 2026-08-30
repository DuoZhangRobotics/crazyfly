"""Compatibility import for the version-2 trajectory mission CLI."""

from tools.trajectory_mission import main


if __name__ == "__main__":
    raise SystemExit(main())
