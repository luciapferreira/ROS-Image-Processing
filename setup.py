from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'trsa_lab1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'video'), glob('video/*.mov')),
        (os.path.join('share', package_name, 'calibration/images'), glob('calibration/images/*')),
        (os.path.join('share', package_name, 'calibration'), glob(os.path.join('calibration', '*ost.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_driver = trsa_lab1.camera_driver:main',
            'camera_reader = trsa_lab1.camera_reader:main',
            'camera_rectifier = trsa_lab1.camera_rectifier:main',
            'camera_calibration_pub = trsa_lab1.camera_calibration_pub:main',
            'image_convert = trsa_lab1.image_convert:main',
            'object_detection = trsa_lab1.object_detection:main',
        ],
    },
)
