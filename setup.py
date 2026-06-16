"""
setup.py
--------
Tells ROS2's ament-python build system how to install this package.
Run `colcon build` from the workspace root to compile.
"""

import os
from glob import glob
from setuptools import setup, find_packages

PACKAGE_NAME = "go2_lidar_detector"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),

    # ── Data files ────────────────────────────────────────────
    # Install the launch files and config so ros2 launch can find them.
    data_files=[
        # Required: registers the package with the ROS2 index
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        # Launch files
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
        # Config files (YAML parameters, thresholds, etc.)
        (f"share/{PACKAGE_NAME}/config", glob("config/*.yaml")),
    ],

    install_requires=["setuptools"],
    zip_safe=True,

    # ── Package metadata ──────────────────────────────────────
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="LiDAR obstacle detector for the Unitree Go2 in Isaac Sim and hardware",
    license="MIT",

    # ── Console entry points ──────────────────────────────────
    # Maps `ros2 run go2_lidar_detector lidar_detector` to main()
    entry_points={
        "console_scripts": [
            "lidar_detector = go2_lidar_detector.lidar_detector:main",
        ],
    },
)
