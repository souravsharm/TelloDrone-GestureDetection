from setuptools import setup

package_name = 'droneController'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'cvzone',
        'numpy',
        'tensorflow',
        'rclpy',
        'djitellopy'
        ],
    zip_safe=True,
    maintainer='sourav',
    maintainer_email='sourav@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "gestures=droneController.gestures:main",
            "tello_drone=droneController.tello_drone:main"
        ],
    },
)
