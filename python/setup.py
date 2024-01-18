#!/usr/bin/env python
from setuptools import setup, find_packages

setup(
    name='touch_typer',
    author='Capstone Group 5',
    packages=find_packages(),
    entry_points={
        'console_scripts': [
            'touch_typer=touch_typer.touch_typer:main',
        ],
    },
    include_package_data=True,
    package_data={
        "": ["*.json"],
    },
    install_requires=[
        'ahrs', 'numpy', 'scipy', 'vpython', 'filterpy', 'bluepy', 'pandas', 'matplotlib',
    ]
)
