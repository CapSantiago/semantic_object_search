from glob import glob
from setuptools import setup

setup(
    name='semantic_search',
    version='0.1.0',
    packages=['semantic_search'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/semantic_search']),
        ('share/semantic_search', ['package.xml']),
        ('share/semantic_search/launch', glob('launch/*.launch.py')),
        ('share/semantic_search/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    entry_points={'console_scripts': [
        'search = semantic_search.search_node:main',
        'detect = semantic_search.detector:main',
        'voice = semantic_search.voice:main',
    ]},
)
