from setuptools import find_packages, setup

package_name='team19_object_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/'+package_name]),
        ('share/'+package_name,['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='burger',
    maintainer_email='ymhaskar@gatech.edu',
    description='Follows the object the camera is trying to track',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_tracking=team19_object_follower.object_tracking:main',
            'rotate_robot2=team19_object_follower.rotate_robot2:main',
            'rotate_robot1=team19_object_follower.rotate_robot1:main',
            'view_image_raw=team19_object_follower.view_image_raw:main',
            'rotate_robot = team19_object_follower.rotate_robot:main',
            'view_image_raw2=team19_object_follower.view_image_raw2:main',
        ],
    },
)
