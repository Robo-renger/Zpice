#!/usr/bin/env python3
from setuptools import setup, find_packages
# this setup is used to make all pakcages inside script directory visible for each other
setup(
    name="cv2",         
<<<<<<< HEAD
    version="0.0.25",
=======
    version="0.0.37",
>>>>>>> e6cfed3dd26a98c416fd417fc53ecaad6592ec9a
    packages=find_packages(where="script"),  
    package_dir={"": "script"},   
    # zip_safe=True,
    install_requires=[
    ],
)
