from glob import glob
from setuptools import setup, find_packages

name = 'tunnel_tcn_test'
setup(
    name=name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml', 'README.md']),
        ('share/' + name + '/launch', glob('launch/*.launch.py')),
        ('share/' + name + '/config', glob('config/*.yaml')),
        ('share/' + name + '/models', glob('models/*')),
        ('share/' + name + '/tools', glob('tools/*.py')),
    ],
    install_requires=['setuptools'], zip_safe=False,
    maintainer='ohslo', maintainer_email='ohslo@example.com', license='Apache-2.0',
    description='Live RGB point-event FP32 tunnel crossing preview for Jetson.',
    entry_points={'console_scripts': ['tunnel_tcn_node = tunnel_tcn_test.node:main', 'verify_model = tunnel_tcn_test.verify:main']},
)
