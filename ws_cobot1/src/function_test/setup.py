from setuptools import find_packages, setup

package_name = 'function_test'

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
    maintainer='osy',
    maintainer_email='seungyeon_oh@hotmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'close_lid_test = function_test.close_lid_test:main',
            'open_lid_test = function_test.open_lid_test:main',
            'force_and_rotate = function_test.force_and_rotate_test:main',
            'moveb_test = function_test.moveb_test:main',
            'mix_drawing_circle = function_test.mix_draw_circle:main',
            'mix_drawing_circle2 = function_test.mix_draw_circle2:main',
            'order_test = function_test.order_test:main',
            'move_to_pose = function_test.move_to_pose:main',
            'diagonal_shake_test = function_test.diagonal_shake_test:main',
            'gripper_status = function_test.gripper_status:main',
        ],
    },
)
