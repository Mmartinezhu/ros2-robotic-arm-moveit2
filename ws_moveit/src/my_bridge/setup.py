from setuptools import find_packages, setup

package_name = 'my_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manuel',
    maintainer_email='159917835+Mmartinezhu@users.noreply.github.com',
    description='Bridge JointState -> Int32MultiArray deltas',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'joint_states_to_delta_int = my_bridge.joint_states_to_delta_int:main',
        ],
    },
)

