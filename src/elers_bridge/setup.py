from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'elers_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This line includes all Python files in the launch directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='naor',
    maintainer_email='naorw@post.bgu.ac.il',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'old_aplication = elers_bridge.old_aplication:main',
        ],
    },
)

