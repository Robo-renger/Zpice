#!/usr/bin/env python3
from setuptools import setup, find_packages
# this setup is used to make all pakcages inside script directory visible for each other
setup(
    name="control2",         
    version="1.1.292",
    packages=find_packages(where="script"),  
    package_dir={"": "script"},   
    install_requires=[
    ],
)
