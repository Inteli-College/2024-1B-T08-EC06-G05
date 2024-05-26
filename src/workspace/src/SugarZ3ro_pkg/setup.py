from setuptools import find_packages, setup

package_name = 'SugarZ3ro_pkg'

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
    maintainer='SugarZ3ro',
    maintainer_email='mario.medeiros@sou.inteli.edu.br',
    description='Pacote de ROS2 para controle do turtlebot3 burger',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "start_moving = SugarZ3ro_pkg.movimentation:main",
        ],
    },
)
