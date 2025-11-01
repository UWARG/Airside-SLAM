from setuptools import setup

package_name = 'mirror_pixhawk'

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
    maintainer='warg',
    maintainer_email='d32kong@uwaterloo.ca',
    description='Minimal pub/sub example',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan = mirror_pixhawk.scan_listener:main',
        ],
    },
)
