from setuptools import find_packages, setup

package_name = "vision_demo"

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wsq",
    maintainer_email="wsq@example.com",
    description="Synthetic RGB-D camera and color detection demo for ROS2.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "synthetic_camera = vision_demo.synthetic_camera:main",
            "detector_node = vision_demo.detector_node:main",
        ],
    },
)
