from glob import glob
from setuptools import find_packages, setup

NAME = 'ir_bev_line_detection'
setup(
    name=NAME, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + NAME]),
        ('share/' + NAME, ['package.xml', 'README.md']),
        ('share/' + NAME + '/launch', glob('launch/*.launch.py')),
        ('share/' + NAME + '/config', glob('config/*.yaml')),
        ('share/' + NAME + '/tools', glob('tools/*.py')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='ohslo', maintainer_email='ohslo@example.com',
    description='Standalone low-speed IR BEV lane-following vehicle test.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'mono_bev_adapter = ir_bev_line_detection.adapter:main',
    ]},
)
