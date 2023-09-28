from setuptools import find_packages, setup

package_name = 'team19_object_follower'

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
    maintainer='burger',
    maintainer_email='ymhaskar@gatech.edu',
    description='Follows the object the camera is trying to track',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_follower = team19_object_follower.object_follower:main'
	    'find_object =  camera_viewer.view_image_raw.main'
        ],
    },
)
