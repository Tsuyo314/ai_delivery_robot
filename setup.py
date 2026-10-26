from setuptools import setup

package_name = 'ai_delivery_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/delivery.launch.py']),
        ('share/' + package_name + '/config', ['config/delivery.yaml']),
        ('share/' + package_name + '/worlds', [
            'worlds/delivery_world.sdf',
            'worlds/delivery_world_physical.sdf',
        ]),
        ('share/' + package_name + '/models', [
            'models/color_classifier.yml']),
        ('share/' + package_name + '/models/delivery_bot', [
            'models/delivery_bot/model.config', 'models/delivery_bot/model.sdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'perception_node = ai_delivery_robot.perception_node:main',
            'planner_node = ai_delivery_robot.planner_node:main',
            'navigation_node = ai_delivery_robot.navigation_node:main',
            'delivery_manager_node = ai_delivery_robot.delivery_manager_node:main',
            'train_color_model = ai_delivery_robot.train_color_model:main',
        ],
    },
)
