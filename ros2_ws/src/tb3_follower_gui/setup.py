from setuptools import setup
import os

package_name = "tb3_follower_gui"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Midhun",
    maintainer_email="midhunr2015@gmail.com",
    description="PyQt5 dashboard for the TB3 follower demo",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dashboard = tb3_follower_gui.dashboard:main",
        ],
    },
)
