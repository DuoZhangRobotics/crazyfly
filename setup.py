from glob import glob

from setuptools import find_packages, setup


PACKAGE_NAME = "crazyfly"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/crazyfly"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/config", glob("config/*.yaml")),
        (f"share/{PACKAGE_NAME}/config/trajectories", glob("config/trajectories/*.yaml")),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "PyYAML", "numpy"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Duo Zhang Robotics",
    maintainer_email="duo@example.invalid",
    description="Safety-gated OptiTrack and Crazyswarm2 experiment control",
    license="MIT",
    entry_points={
        "console_scripts": [
            "crazyfly_safety_gateway = crazyfly.safety_gateway:main",
            "crazyfly_mock_stack = crazyfly.mock_stack:main",
            "crazyfly_validate_config = crazyfly.config_validator:main",
            "crazyfly_trajectory = tools.trajectory_mission:main",
            "crazyfly_trajectory_mission = tools.trajectory_mission:main",
            "crazyfly_experiment_logger = crazyfly.experiment_logger:main",
        ],
    },
)
