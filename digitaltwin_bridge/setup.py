from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup_args = generate_distutils_setup(
    packages=['digitaltwin_bridge'],
    package_dir={'': 'src'},
    install_requires=['paho-mqtt']
    # install_requires=['numpy']
)

setup(**setup_args)
