from setuptools import find_packages, setup

package_name = 'joy_udp_relay'

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
    maintainer='btja',
    maintainer_email='brian.w.tjahjadi@gmail.com',
    description='This node transforms joystick states from UDP `127.0.0.1:5005` to the `/joy` topic.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'start = joy_udp_relay.joy_udp_relay:main',
        ],
    },
)
