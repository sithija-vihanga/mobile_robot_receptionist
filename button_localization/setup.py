from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'button_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/visualization.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sadeep',
    maintainer_email='sadeepm20@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_estimation     = button_localization.line_estimation:main',
            'button_localization = button_localization.button_localization:main',
            'button_detection    = button_localization.button_detection:main',
            'yolo_node           = button_localization.yolo_node:main' ,
            'visualizer_node     = button_localization.visualizer_node:main'         
        ],
    },
)
