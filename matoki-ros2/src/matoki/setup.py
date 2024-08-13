import os
from glob import glob
from setuptools import setup

package_name = 'matoki'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all .py files in the launch directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Description of your package',
    license='Your License',
    extras_require={
        'dev': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'rc_in = matoki.rc_in:main',
            # Add other nodes here if needed
        ],
    },
)


