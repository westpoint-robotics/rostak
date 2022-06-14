from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rostak'],
    scripts=['scripts/rostak_bridge', 'scripts/roscot_fix'],
    package_dir={'': 'src'}
)

setup(**d)