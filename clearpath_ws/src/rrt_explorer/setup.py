from setuptools import find_packages, setup

package_name = 'rrt_explorer'

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
    maintainer='elizabeth_s',
    maintainer_email='elizabeth_s@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
           'rrt_node = rrt_explorer.rrt_node:main',
           'rrt_detect = rrt_explorer.rrt_detect:main'
        ],
    },
)
