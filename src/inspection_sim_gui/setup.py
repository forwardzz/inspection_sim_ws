from glob import glob
import os

from setuptools import setup


package_name = "inspection_sim_gui"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "assets"), glob("assets/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zjy",
    maintainer_email="zjy@example.com",
    description="PyQt5 control panel for inspection robot simulation",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "inspection_sim_gui = inspection_sim_gui.main:main",
        ],
    },
)
