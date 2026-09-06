from setuptools import find_packages, setup

package_name = "cs625_motion_adapter"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CS625 Active Perception",
    maintainer_email="cs625@example.invalid",
    description="MoveIt planning-only reachability filtering.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "reachability_filter = cs625_motion_adapter.reachability_filter:main",
            "sim_view_executor = cs625_motion_adapter.sim_view_executor:main",
            "p7_arm_motion_adapter = cs625_motion_adapter.p7_arm_motion_adapter:main",
            "real_preflight = cs625_motion_adapter.real_preflight:main",
            "real_readiness_monitor = cs625_motion_adapter.real_readiness_monitor:main",
            "real_view_executor = cs625_motion_adapter.real_view_executor:main",
        ],
    },
)
