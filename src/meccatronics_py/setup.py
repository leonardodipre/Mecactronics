from setuptools import setup
from setuptools import find_packages

package_name = 'meccatronics_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A simple ROS2 package for testing',
    license='License declaration',
    entry_points={
        'console_scripts': [
            'simple_node = meccatronics_py.simple_node:main',
            'turtle_square_path = meccatronics_py.turtle_square_path:main',
            'ros2_goforward = meccatronics_py.ros2_goforward:main',
            'es3_task_1 = meccatronics_py.es3_task_1:main',
            'follow_path_pid = meccatronics_py.follow_path_pid:main',
            'follow_path_function = meccatronics_py.follow_path_function:main',
            
          

        ],
    },
)
