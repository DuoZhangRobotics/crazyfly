#!/usr/bin/env python3
"""Run the selected-fleet hover helper."""

try:
    from .all_drone_hover import main
except ImportError:  # Direct execution from the tools directory.
    from all_drone_hover import main


if __name__ == "__main__":
    raise SystemExit(main())
