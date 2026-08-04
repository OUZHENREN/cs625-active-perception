from setuptools import find_packages, setup
package_name = 'cs625_target_perception'
setup(
    name=package_name, version='0.0.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='yff', maintainer_email='yff@todo.todo',
    description='Minimal target pose relay for P2',
    license='TODO',
    entry_points={
        'console_scripts': [
            'target_pose_relay = cs625_target_perception.target_pose_relay:main',
        ],
    },
)
