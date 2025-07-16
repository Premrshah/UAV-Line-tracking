
from setuptools import find_packages, setup

package_name = 'uav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','opencv-python'],
    zip_safe=True,
    maintainer='firefox',
    maintainer_email='firefox@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_reg = uav.linear_reg:main',
            'image_publisher = uav.image_publisher:main'
        ],
    },
)
