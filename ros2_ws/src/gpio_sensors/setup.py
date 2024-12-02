from setuptools import setup

package_name = 'gpio_sensors'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name]
    install_requires=['setuptools', 'rclpy', 'RPi.GPIO'],
    zip_safe=True,
    maintainer='spiderbot',
    maintainer_email='ellahick@buffalo.edu',
    description='A package to monitor GPIO sensors for foot states',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gpio_sensor_node = gpio_sensors.gpio_sensor_node:main'
        ],
    },
)
