from setuptools import find_packages, setup
import os
from glob import glob

package_name = "ball_balance_controller"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        # Install all launch files
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        # Install all config files
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools", "ikpy"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="PID ball-on-plate controller for UR7e with IK backend",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ball_balance_controller = ball_balance_controller.controller:main",
        ],
    },
)
