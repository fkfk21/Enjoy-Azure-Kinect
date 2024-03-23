from setuptools import find_packages, setup

package_name = 'enjoy_azure_kinect'

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
    maintainer='Kenji Fukuda',
    maintainer_email='fk.kenji.21@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hark_executor_node = scripts.online_sound_localization:main',
            'compress_image_node = scripts.compress_image:main'
        ],
    },
)
