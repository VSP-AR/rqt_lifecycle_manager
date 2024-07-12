from setuptools import setup
import os

package_name = 'rqt_lifecycle_manager'

setup(
    name=package_name,
    version='0.0.0',
    package_dir={'': 'src'},
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/resource', ['resource/RosLifecycleManager.ui']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
        ('lib/' + package_name, ['scripts/rqt_lifecycle_manager']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Dirk Thomas',
    maintainer='ipa-vsp',
    maintainer_email='vishnu.pbhat93@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'rqt_lifecycle_manager provides GUI plugin for visulazing lifecycle node state and change it'
    ),
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rqt_lifecycle_manager = rqt_lifecycle_manager.main:main',
        ],
    },
)
