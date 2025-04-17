#!/usr/bin/env python3
from setuptools import setup, find_packages
# This setup is used to make (test, script, mock and ms5837) visible for each other
setup(
    name='control',  # Replace with your package name
    version='1.1.279',
    packages=find_packages(where='src'),  # Automatically find packages under 'src'
    package_dir={'': 'src'},  # Specify that packages are located in 'src'
    install_requires=[],  # Add any Python dependencies here
    # zip_safe=True,  # Make the package compatible with ROS
    # pip3 install -e .
)
