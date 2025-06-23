from setuptools import find_packages, setup

package_name = 'pcd_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=['pcd_tools'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ainhoaarnaiz',
    maintainer_email='ainhoaarnaiz@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'pcd_publisher = pcd_tools.pcd_publisher:main',
        ],
    },
)
