from setuptools import find_packages, setup

package_name = 'camera_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/single_camera.launch.py']),
        ('share/' + package_name + '/launch', ['launch/multi_camera.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mgrzybek',
    maintainer_email='mgrzybek@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = camera_package.camera_node:main',
            'processing_node = camera_package.processing_node:main',
        ],
    },
)
