from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_application_code_cannot_bypass_the_safety_gateway() -> None:
    for path in (ROOT / "crazyfly").glob("*.py"):
        source = path.read_text(encoding="utf-8")
        assert "import crazyflie_py" not in source, f"direct Crazyswarm API import in {path.name}"
        assert "from crazyflie_py" not in source, f"direct Crazyswarm API import in {path.name}"
