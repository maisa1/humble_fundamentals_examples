from setuptools import find_packages, setup

package_name = 'turtle_controller'

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
    maintainer='maisa',
    maintainer_email='maisa@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ["test_node = turtle_controller.python_skeleton:main",
                            "oop_version= turtle_controller.python_skeleton_oop:main",
                            "oop_timer= turtle_controller.python_skeleton_oop_timer:main",
                            "publisher_test= turtle_controller.publisher_test:main",
                            "subscriber_test= turtle_controller.subscriber_test:main",
                            "number_publisher=turtle_controller.number_publisher:main",
                            "add_two_ints_server=turtle_controller.add_two_ints_server:main",
                            "add_two_ints_client_no_oop=turtle_controller.add_two_ints_client_no_oop:main",
                            "add_two_ints_client=turtle_controller.add_two_ints_client:main",
                            "hardware_status_publisher=turtle_controller.hardware_status_publisher:main",
                            "led_panel_state=turtle_controller.led_panel:main",
                            "battery_node=turtle_controller.battery_node:main",
                            "draw_circle=turtle_controller.draw_circle:main",
                            "circular_spiral=turtle_controller.circular_spiral:main",
                            "turtle_polygon=turtle_controller.turtle_polygon:main",
                            "turtle_polygon_without_orientation=turtle_controller.turtle_polygon_without_orientation:main"


        ], 
    },
)
