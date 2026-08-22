import os
from glob import glob

from setuptools import setup

package_name = "togsim_hmi"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "static"), glob("static/*.*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Med Nadim Touil",
    maintainer_email="ntouil87@gmail.com",
    description="Operator HMI for the tog-sim cell (FastAPI + vanilla JS).",
    license="Apache-2.0",
    entry_points={"console_scripts": ["hmi_server = togsim_hmi.server:main"]},
)
