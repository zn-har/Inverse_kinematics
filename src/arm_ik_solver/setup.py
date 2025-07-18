from setuptools import find_packages, setup

package_name = 'arm_ik_solver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	('share/' + package_name + '/urdf', ['urdf/five_dof_arm.urdf']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zenhar',
    maintainer_email='zenharmuhammed00@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "kinematics=arm_ik_solver.arm_ik_solver_kinematics:main",
            "angles=arm_ik_solver.arm_ik_solver_angle:main",
            "target = arm_ik_solver.target_publisher:main"
        ],
    },
)
