#!/usr/bin/env python

try:
    from setuptools import setup

except ImportError:
    from distutils.core import setup

setup(
    name='obstacle-path-finder',
    version='0.1-beta.1',
    author=('Unmaned Aerial Vehicle Team | The University of Texas at Austin' +
            ', Computational Engineering Association | ' +
            'The University of Texas at Austin'),
    url='https://github.com/uav-team-ut/Obstacle-Path-Finder',
    packages=['path_finder'],
    scripts=[],
    install_requires=[]
)
