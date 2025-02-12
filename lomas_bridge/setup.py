from setuptools import find_packages, setup

package_name = 'lomas_bridge'

setup(
    name = package_name,
    version = '0.0.1',
    packages = find_packages(exclude=['test']),
    data_files = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires = ['setuptools'],
    zip_safe = True,
    maintainer = 'Andreas Persson',
    maintainer_email = 'andreas.persson@oru.se',
    description = 'ROS2 package bridge for the LOMAS project',
    license = 'MIT',
    tests_require = ['pytest'],
    entry_points = {
        'console_scripts': [
            'machine = lomas_bridge.machine:main',
            'converter = lomas_bridge.converter:main'
        ],
    },
)
