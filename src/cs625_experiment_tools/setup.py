from setuptools import find_packages, setup


package_name = "cs625_experiment_tools"

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
    description="Reproducible P5 paired experiment and metric export tools.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "run_paired_experiment = cs625_experiment_tools.paired_experiment:main",
            "summarize_p4_matrix = cs625_experiment_tools.p4_matrix_summary:main",
        ],
    },
)
