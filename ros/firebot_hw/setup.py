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
            'servo_node = firebot_hw.servo_node:main',
            'hw_node = firebot_hw.hw_node:main',
            'mic_node = firebot_hw.mic_node:main',
        ],
    },
)
