import os
from glob import glob
from setuptools import setup

package_name = "gz_waves_bridge"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "models"), glob("models/*.xacro")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Rhys Mainwaring",
    maintainer_email="rhys.mainwaring@me.com",
    description="ROS2 bridge for gz-waves",
    license="GPL-3.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "body_response_publisher = gz_waves_bridge.body_response_publisher:main",
        ],
    },
)
