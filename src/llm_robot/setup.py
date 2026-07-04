from setuptools import find_packages, setup

package_name = 'llm_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/simulation.launch.py']),
    ('share/' + package_name + '/worlds', ['worlds/my_world.sdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cemal',
    maintainer_email='cemalgurselkar@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'camera = llm_robot.camera:main',
        'detector = llm_robot.detection:main',
        'scanner = llm_robot.scanner:main',
        'navigator = llm_robot.navigator:main',
        'llm_inference = llm_robot.llm_inference:main',
        'arm_controller = llm_robot.arm_controller:main',
    ],
},
)
