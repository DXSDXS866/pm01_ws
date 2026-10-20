from glob import glob

from setuptools import find_packages, setup

package_name = "yolo_person_3d"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/person_3d.launch.py", "launch/bringup.launch.py"]),
        ("share/" + package_name + "/weights", glob("weights/*.engine")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    scripts=["scripts/person_3d_node"],
)
