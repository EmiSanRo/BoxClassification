from setuptools import find_packages, setup

package_name = 'aruco_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['config.json'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emy',
    maintainer_email='emy@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector = aruco_pkg.aruco_reto_v3:main',
            'recorte = tu_paquete.recorte:main',
        ],
    },
)
