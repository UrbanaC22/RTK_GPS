from setuptools import find_packages, setup

package_name = 'rtk_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pynmeagps'],
    zip_safe=True,
    maintainer='urbana',
    maintainer_email='urbana@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'nmea_node= rtk_pkg.main:main',
            'conv_coord=rtk_pkg.coordinate:main'
        ],
    },
)
