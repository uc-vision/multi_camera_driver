from catkin_pkg.python_setup import generate_distutils_setup
from distutils.core import setup

setup_args = generate_distutils_setup(
    packages=["spinnaker_camera_driver_helpers"],
    package_dir={
        "spinnaker_camera_driver_helpers": 'src/spinnaker_camera_driver_helpers'
    },
    install_requires=['spinnaker_python', 'yaml', 'camera_geometry_python', 'pyturbojpeg', 'disable_gc', 'py_structs']
)

setup(**setup_args)
