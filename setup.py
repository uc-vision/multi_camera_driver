import os
from glob import glob
from setuptools import setup

package_name = 'multi_camera_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),


        (os.path.join('share', package_name, 'launch'), 
          glob(os.path.join('launch', '*launch.[pxy][yma]*'))),

    ],
    install_requires=[
        'setuptools', 
        'spinnaker_python', 
        'camera_geometry_python',
        'pydispatch',
        'pyturbojpeg', 
        'disable_gc', 
        'py_structs'],
    zip_safe=True,
    maintainer='mma484',
    maintainer_email='matthew.mattar@canterbury.ac.nz',
    description='The ros_camera_array_driver package',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
