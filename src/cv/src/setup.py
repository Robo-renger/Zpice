#!/usr/bin/env python3
from setuptools import setup, find_packages
# this setup is used to make all pakcages inside script directory visible for each other
setup(
    name="cv2",         
    version="0.0.30",
    packages=find_packages(where="script"),  
    package_dir={"": "script"},   
    # zip_safe=True,
    install_requires=[
    ],
)
