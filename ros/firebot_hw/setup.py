from setuptools import setup

package_name = 'firebot_hw'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='erik@orjehag.se',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vl53l1x_node = firebot_hw.vl53l1x_node:main',
            'tpa81_node = firebot_hw.tpa81_node:main',
            'servo_node = firebot_hw.servo_node:main',
            'motor_node = firebot_hw.motor_node:main',
        ],
    },
)
