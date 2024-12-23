from setuptools import setup, find_packages

setup(
    name='control',  # Replace with your package name
    version='0.0.0',
    packages=find_packages('src'),  # Automatically find packages under 'src'
    package_dir={'': 'src'},  # Specify that packages are located in 'src'
    install_requires=[],  # Add any Python dependencies here
    zip_safe=True,  # Make the package compatible with ROS
    # pip3 install -e .
)
