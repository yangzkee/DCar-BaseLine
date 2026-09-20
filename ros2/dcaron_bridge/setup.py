from glob import glob
from setuptools import setup

setup(
    name="dcaron_bridge",
    version="0.2.0",
    packages=["dcaron_bridge"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/dcaron_bridge"]),
        ("share/dcaron_bridge", ["package.xml"]),
        ("share/dcaron_bridge/launch", glob("launch/*.launch.py")),
        ("share/dcaron_bridge/config", glob("config/*.yaml")),
        ("share/dcaron_bridge/examples", glob("examples/*.yaml")),
    ],
    install_requires=["setuptools", "pyserial>=3.5,<4"],
    zip_safe=True,
    maintainer="yangzkee",
    maintainer_email="220290149+yangzkee@users.noreply.github.com",
    description="DcarON DFLink V3 serial bridge for ROS 2",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["dcaron_bridge = dcaron_bridge.node:main",
                                      "motion_sequence = dcaron_bridge.sequence:main"]},
)
