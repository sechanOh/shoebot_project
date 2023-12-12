from setuptools import find_packages, setup

package_name = 'ai_manipulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        'ai_manipulation': ['utils/*', 'utils/models/*', 'utils/utils/*'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sechan',
    maintainer_email='sechan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'make_my_model = ai_manipulation.script.make_my_model:main',
            'run_my_model = ai_manipulation.script.run_my_model:main',
            'six_dxl_model = ai_manipulation.script.six_dxl_model:main',
            'six_dxl_run = ai_manipulation.script.six_dxl_run:main',
            'testing = ai_manipulation.script.testing:main',
            'test_seokwon = ai_manipulation.script.test_seokwon:main',
            'shoe_drop_point_gen = ai_manipulation.script.shoe_drop_point_gen:main',
        ],
    },
)
