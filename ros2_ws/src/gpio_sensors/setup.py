from setuptools import setup

package_name = 'gpio_sensors'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/gpio_sensors']),
        ('share/gpio_sensors', ['package.xml']),
        ('share/gpio_sensors/launch', ['launch/gpio_sensors.launch.py'])
],
    install_requires=['setuptools', 'RPi.GPIO'],
    zip_safe=True,
    maintainer='Ella Hicks',
    maintainer_email='ellahick@buffalo.edu',
    description='A package for monitoring GPIO sensors.',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'gpio_sensor_node = gpio_sensors.gpio_sensor_node:main',
        ],
    },

)
