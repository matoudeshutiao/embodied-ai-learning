from glob import glob

from setuptools import find_packages, setup

package_name = "vla_bridge"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            glob("launch/*.launch.py"),
        ),
        (
            "share/" + package_name + "/rviz",
            glob("rviz/*.rviz"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wsq",
    maintainer_email="wsq@example.com",
    description="VLA and real robot bridge skeleton for ROS2.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mock_policy = vla_bridge.mock_policy:main",
            "safety_filter = vla_bridge.safety_filter:main",
            "mock_trajectory_server = vla_bridge.mock_trajectory_server:main",
            "safe_executor = vla_bridge.safe_executor:main",
        ],
    },
)
