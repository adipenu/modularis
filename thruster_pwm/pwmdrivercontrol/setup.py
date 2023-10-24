from setuptools import find_packages, setup
from glob import glob

package_name = 'pwmdrivercontrol'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')), # This line for launch files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aprilab',
    maintainer_email='aprilab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pwm_control_node = pwmdrivercontrol.pwm_control_node:main'
        ],
    },
)
