from glob import glob
import os

from setuptools import find_packages, setup


PACKAGE_NAME = "arena_3d_v1"


setup(
    name=PACKAGE_NAME,
    version="1.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        (os.path.join("share", PACKAGE_NAME, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy", "pyyaml"],
    zip_safe=True,
    maintainer="Pudu planner research",
    maintainer_email="robot@example.com",
    description="Independent dynamic 3D-V1 layered planner research package",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "three_d_v1_stage_a = arena_3d_v1.stage_a_benchmark:main",
            "three_d_v1_real_stage_a = arena_3d_v1.real_stage_a_benchmark:main",
            "three_d_v1_stage_b_smoke = arena_3d_v1.stage_b_smoke:main",
            "three_d_v1_r1_profile = arena_3d_v1.r1_profile:main",
            "three_d_v1_r1_stage_a = arena_3d_v1.r1_stage_a:main",
            "three_d_v1_r1_soak = arena_3d_v1.r1_soak:main",
            "three_d_v1_r1_stage_b_gate = arena_3d_v1.r1_stage_b_gate:main",
        ],
    },
)
