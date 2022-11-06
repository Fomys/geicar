from setuptools import setup

package_name = 'web_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrea',
    maintainer_email='andreaperezfdez@gmail.com',
    description='Web interface to control PostCar',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_interface = web_interface.main',
        ],
    },
)
