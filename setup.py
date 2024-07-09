from setuptools import find_packages, setup

package_name = 'rqt_lifecycle_manager'

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
    maintainer='ipa-vsp',
    maintainer_email='vishnu.pbhat93@gmail.com',
    description='rqt_lifecycle_manager provides GUI plugin for visulazing lifecycle node state and change it',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_cli = rqt_lifecycle_manager.test_cli:main',
        ],
    },
)
