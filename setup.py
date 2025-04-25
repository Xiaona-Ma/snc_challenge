from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'snc_challenge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'navigation_node_executable = {package_name}.navigation_node:main',
            f'detection_node = {package_name}.detection_node:main',
            f'tracking_node_executable = {package_name}.tracking_node:main',
            'best_effort_repeater = aiil_rosbot_demo.best_effort_repeater:main',
        ],
    },
)
