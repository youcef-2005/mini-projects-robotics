from setuptools import find_packages, setup

package_name = 'mini_project_11'

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
    description='OpenCV Color Tracking Node for ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_tracker = src.color_tracking_node:main',
        ],
    },
)
