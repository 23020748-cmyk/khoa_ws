from glob import glob
import os

from setuptools import setup

package_name = "ur3_llm_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Lục Văn Khoa",
    maintainer_email="khoa@todo.todo",
    description="Validated natural-language task planning and MoveIt 2 skills for UR3/UR3e.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "llm_planner = ur3_llm_control.llm_planner:main",
            "skill_executor = ur3_llm_control.skill_executor:main",
        ],
    },
)
