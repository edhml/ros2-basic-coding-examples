from setuptools import setup

package_name = 'laser_scan_publisher_py'

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
    maintainer='adlinker',
    maintainer_email='adlinker@adlink.com',
    description='Topic pub demo',
    license='ADLINK',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laser_scan_pub_py = laser_scan_publisher_py.laser_scan_pub_py:main'
        ],
    },
)
