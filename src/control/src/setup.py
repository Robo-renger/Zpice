#!/usr/bin/env python3
from setuptools import setup, find_packages

setup(
    name="control",         
    version="0.1",
    packages=find_packages(where="script"),  
    package_dir={"": "script"},   
    install_requires=[
    ],
)
