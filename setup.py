import os
from glob import glob
from setuptools import find_packages, setup
import os

home_path = os.path.expanduser("~")
executable_path = os.path.join(home_path, '.virtualenvs', 'rosmav', 'bin', 'python')
package_name = "rosmav"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ms.ibnseddik@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bluerov2_hardware_interface = rosmav.bluerov2_hardware_interface:main",
            "bluerov2_camera_interface = rosmav.bluerov2_camera_interface:main",
            "bluerov2_simulation_interface = rosmav.bluerov2_simulation_interface:main",
        ],
    },
    options={
        'build_scripts': {
            'executable': executable_path,
        }
    },
)
