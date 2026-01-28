from setuptools import find_packages, setup

package_name = 'ur5_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kanat',
    maintainer_email='kanat@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mpc_control_exe = ur5_control.mpc_ur5:main',  
            'pd_control_exe = ur5_control.pd_ur5:main',
            'mpc_constrained_control_exe = ur5_control.mpc_constrained_ur5:main',
            'waypoint_publisher_exe = ur5_control.waypoint_publisher:main',
        ],
    },
)
