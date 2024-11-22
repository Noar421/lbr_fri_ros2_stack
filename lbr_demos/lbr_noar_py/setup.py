from setuptools import find_packages, setup

package_name = 'lbr_noar_py'

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
    maintainer='noar',
    maintainer_email='noar@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             "traj_planner = lbr_noar_py.moveit_traj_planner:main",
             "sequenced_motion_cart = lbr_noar_py.sequenced_motion_cart:main",
             "sequenced_motion_joint = lbr_noar_py.sequenced_motion_joint:main",
             "unity_moveit_bridge_ptp = lbr_noar_py.unity_moveit_bridge_ptp:main",
             "unity_moveit_bridge = lbr_noar_py.unity_moveit_bridge:main",
       ],
    },
)
