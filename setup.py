from setuptools import setup

package_name = 'francor_timeout'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='m1ch1',
    maintainer_email='m1ch1@todo.todo',
    description='TODO: Package description',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'timeout_nav_node = francor_timeout.timeout_nav_node:main'
        ],
    },
)
