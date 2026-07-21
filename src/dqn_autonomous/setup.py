import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dqn_autonomous'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name,
        package_name + '.node',
        package_name + '.map'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'urdf', 'meshes'), glob('urdf/meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ris',
    maintainer_email='ris@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ui_node = dqn_autonomous.node.ui_node:main',
            'driver_node = dqn_autonomous.node.driver_node:main',
            'environment_node = dqn_autonomous.node.environment_node:main',
        ],
    },
)
