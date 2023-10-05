from setuptools import find_packages, setup

package_name = 'lidar_test'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'print_lidar_output = lidar_test.print_lidar_output:main',
            'detect_box = lidar_test.detect_box:main',
            'publish_lidar_with_color = lidar_test.publish_lidar_with_color:main'

        ],
    },
)
