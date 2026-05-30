from setuptools import setup

package_name = 'demo_python_pkg'

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
    maintainer='mi',
    maintainer_email='mi@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "striker = demo_python_pkg.ball_kicker:main",
            "kick = demo_python_pkg.ball_kicker:main",
            "max_forward = demo_python_pkg.max_forward:main",
            "max_spin = demo_python_pkg.max_spin:main",
            "sit = demo_python_pkg.sit_down:main",
        ],
    },
)
