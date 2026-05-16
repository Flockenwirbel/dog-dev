from setuptools import setup

package_name = 'demo_python_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'demo_python_pkg': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['../resource/' + package_name]),
        ('share/' + package_name, ['../package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mi',
    maintainer_email='mi@todo.todo',
    description='CyberDog goalkeeper',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goalkeeper = demo_python_pkg.run_goalkeeper:main',
        ],
    },
)
