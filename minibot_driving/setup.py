from setuptools import find_packages, setup

package_name = 'minibot_driving'

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
    maintainer='kimdu',
    maintainer_email='dongterm1017@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'test_driving = minibot_driving.drive',
        'service = minibot_driving.service:main',
        'client = minibot_driving.client:main'
        
         
        ],
    },
)
