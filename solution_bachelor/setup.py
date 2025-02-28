from setuptools import setup

package_name = 'solution_bachelor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saladwithgrass',
    maintainer_email='egod09080908@yandex.ru',
    description='Solution for IProfi 2025',
    license='i dont know, BSD or GPL probably',
    tests_require=['pytest'],
    entry_points={},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', ['msg/TwistStampedArray.msg']),
        ('share/' + package_name + '/msg', ['msg/Pose2DArray.msg']),
    ],
    # Include message files when building
    package_data={'': ['msg/*.msg']},
    # Call generate_messages to generate message types
    package_dir={'': '.'},
    setup_requires=['rosidl_default_generators', 'rosidl_python'],
    zip_safe=True,
)
