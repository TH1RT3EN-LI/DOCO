from glob import glob
import os

from setuptools import setup


package_name = "duojin01_bringup"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="litianshun",
    maintainer_email="litianshun.cn@gmail.com",
    description="Collaborative bringup package",
    license="GPL-3.0-only",
    entry_points={"console_scripts": []},
)
