from setuptools import find_packages, setup


package_name = "cs625_task_orchestrator"

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
    description="P7 task-level grasp evidence contract and recorder.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "p7_episode_recorder = cs625_task_orchestrator.p7_episode_recorder:main",
        "p7_gripper_adapter = cs625_task_orchestrator.p7_gripper_adapter:main",
        "p7_attachment_adapter = cs625_task_orchestrator.p7_attachment_adapter:main",
    ]},
)
