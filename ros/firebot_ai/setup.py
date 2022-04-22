from setuptools import setup

package_name = 'firebot_ai'

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
    maintainer='erik',
    maintainer_email='erik@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ai_node = firebot_ai.ai_node:main',
            'loc_node = firebot_ai.loc_node:main'
        ],
    },
)
