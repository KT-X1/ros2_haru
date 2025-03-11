from setuptools import setup

package_name = 'harurobo_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='altair',
    maintainer_email='altair@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'can_node = harurobo_pkg.can_node:main',
            'web_socket_node = harurobo_pkg.web_socket_node:main',
            'planning_node = harurobo_pkg.planning_node:main',
        ],
    },
)
