from setuptools import find_packages
from setuptools import setup

package_name='racecar_mn'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('config', ['config/params.yaml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],

    author='Andrew Fishberg',
    author_email='fishberg@mit.edu',
    maintainer='Andrew Fishberg',
    maintainer_email='fishberg@mit.edu',
    url="https://github.com/mit-bwsi-racecar-ms-DEVEL/racecar-mn",
    keywords=['ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT',
        'Programming Language :: Python',
        'Topic :: Software Development',
        ],
    description='base layer utilities for a racecar'
)