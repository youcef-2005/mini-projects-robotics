from setuptools import find_packages, setup

package_name = 'mini_project_04'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Youcef',
    maintainer_email='youcef@todo.todo',
    description='ROS 2 Client/Service Nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service_server = src.service_server:main',
            'service_client = src.service_client:main',
        ],
    },
)
