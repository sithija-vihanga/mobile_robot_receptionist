from setuptools import find_packages, setup

package_name = 'button_localization'

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
    maintainer='sadeep',
    maintainer_email='sadeepm20@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_estimation     = button_localization.line_estimation:main',
            'button_localization = button_localization.button_localization:main',
            'button_detection    = button_localization.button_detection:main'
        ],
    },
)
