from setuptools import setup

package_name = 'head_solver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.com',
    description='Head parallel mechanism IK/FK solver node',
    license='TODO',
    entry_points={
        'console_scripts': [
            'solver_node = head_solver.solver_node:main',
            'torque_control_node = head_solver.torque_control_node:main',
        ],
    },
)
