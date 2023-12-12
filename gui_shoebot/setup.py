from setuptools import find_packages, setup

package_name = 'my_new_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={
        'my_new_package': ['*'],
    },
    install_requires=['setuptools',
                      ],
    zip_safe=True,
    maintainer='jin',
    maintainer_email='jin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Gui_ShoeBot=my_new_package.Gui_ShoeBot:main',
          
            'Self_Test_Minibot=my_new_package.Self_Test_MiniBot:main',
            'Self_Test_Roboarm=my_new_package.Self_Test_Roboarm:main',
            
            'Self_Test_faceid=my_new_package.Self_Test_faceid:main',
        ],
    },
)
