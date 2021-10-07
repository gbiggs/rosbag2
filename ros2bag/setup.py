from setuptools import find_packages
from setuptools import setup

package_name = 'ros2bag_backport'

setup(
    name=package_name,
    version='0.9.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['ros2cli'],
    zip_safe=True,
    author='Karsten Knese',
    author_email='karsten@osrfoundation.org',
    maintainer='Karsten Knese',
    maintainer_email='karsten@osrfoundation.org',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='Entry point for rosbag in ROS 2',
    long_description="""\
The package provides the rosbag command for the ROS 2 command line tools.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'bag_bp = ros2bag_backport.command.bag:BagCommand',
        ],
        'ros2cli.extension_point': [
            'ros2bag_backport.verb = ros2bag_backport.verb:VerbExtension',
        ],
        'ros2bag_backport.verb': [
            'info = ros2bag_backport.verb.info:InfoVerb',
            'list = ros2bag_backport.verb.list:ListVerb',
            'play = ros2bag_backport.verb.play:PlayVerb',
            'record = ros2bag_backport.verb.record:RecordVerb',
            'reindex = ros2bag_backport.verb.reindex:ReindexVerb'
        ],
    }
)
