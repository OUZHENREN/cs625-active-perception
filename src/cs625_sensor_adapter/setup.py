from setuptools import setup

package_name = "cs625_sensor_adapter"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "point_cloud_relay = cs625_sensor_adapter.point_cloud_relay:main",
            "rgbd_sensor_adapter = cs625_sensor_adapter.point_cloud_relay:main",
        ],
    },
)
