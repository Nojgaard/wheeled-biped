from setuptools import find_packages, setup

package_name = "wobl_gui"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/tune_pid_gains.launch.py"]),
        ("share/" + package_name + "/launch", ["launch/tune_lqr_gains.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nikolai Nojgaard",
    maintainer_email="nnoej10@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "tune_gains_node = wobl_gui.tune_gains_node:main",
            "tune_lqr_node = wobl_gui.tune_lqr:main"
        ],
    },
)
