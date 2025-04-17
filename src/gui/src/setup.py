#!/usr/bin/env python3
from setuptools import setup, find_packages
# this setup is used to make all pakcages inside script directory visible for each other
setup(
    name="gui2",         
<<<<<<< HEAD
    version="0.0.140",
=======
    version="0.0.140",
>>>>>>> 6f7786ade4db56176ece1001047e47f71303bd2e
    packages=find_packages(where="script"),  
    package_dir={"": "script"},   
    install_requires=[
    ],
)
