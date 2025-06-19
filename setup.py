from setuptools import setup

package_name = 'sensor_fusion_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/sensor_fusion_node']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Sensor fusion node for emergency braking using LIDAR',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fusion_node = sensor_fusion_node.fusion_node:main',
        ],
    },
)
