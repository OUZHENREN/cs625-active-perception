from setuptools import find_packages, setup

package_name = "cs625_view_evaluation"
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
    description="Shared baseline selection over reachable candidates.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "baseline_selector = cs625_view_evaluation.baseline_selector:main",
        "sim_episode_coordinator = cs625_view_evaluation.sim_episode_coordinator:main",
    ]},
)
