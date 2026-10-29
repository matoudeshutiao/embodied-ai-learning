import os
from glob import glob
from setuptools import setup

package_name = "turtle_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wsq",
    maintainer_email="wsq@example.com",
    description="ROS2 turtle publisher, service and launch demo",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "turtle_circle = turtle_control.turtle_circle:main",
            "pose_listener = turtle_control.pose_listener:main",
        ],
    },
)
