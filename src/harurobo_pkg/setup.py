from setuptools import find_packages, setup

package_name = 'harurobo_pkg'

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
    maintainer='taichi',
    maintainer_email='taichi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_socket_node = harurobo_pkg.web_socket_node:main',
            'serial_send_node = harurobo_pkg.serial_send_node:main',
            'serial_receive_node = harurobo_pkg.serial_receive_node:main',
            'planning_node = harurobo_pkg.planning_node:main'
        ],
    },
)
