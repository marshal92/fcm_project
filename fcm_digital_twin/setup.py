import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'fcm_digital_twin'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # === АВТОМАТИЧЕСКАЯ УСТАНОВКА ТВОИХ ФАЙЛОВ ===
        # Лаунчи
        (os.path.join('share', package_name, 'launch/simulation'), glob('launch/simulation/*.launch.py')),
        (os.path.join('share', package_name, 'launch/real'), glob('launch/real/*.launch.py')),
        (os.path.join('share', package_name, 'launch/core'), glob('launch/core/*.launch.py')),
        
        # Конфиги
        (os.path.join('share', package_name, 'config/nav2'), glob('config/nav2/*.yaml')),
        (os.path.join('share', package_name, 'config/slam'), glob('config/slam/*.yaml')),
        (os.path.join('share', package_name, 'config/ekf'), glob('config/ekf/*.yaml')),
        (os.path.join('share', package_name, 'config/foxglove'), glob('config/foxglove/*.json')),
        (os.path.join('share', package_name, 'config/rviz'), glob('config/rviz/*.rviz')),
        
        # Среда (Миры и Карты)
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oleksandr',
    maintainer_email='proskurin1408@gmail.com',
    description='Digital Twin for FCM extraction',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
