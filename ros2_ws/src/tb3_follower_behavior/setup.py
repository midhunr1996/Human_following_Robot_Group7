from setuptools import setup, find_packages
from glob import glob
import os

package_name = "tb3_follower_behavior"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "config"),
         glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Midhun",
    maintainer_email="midhunr2015@gmail.com",
    description="py_trees behavior tree for TB3 person following",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "follower_bt_node = tb3_follower_behavior.follower_bt_node:main",
        ],
    },
)
