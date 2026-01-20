import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'babo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 런치 파일 등록
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gom',
    maintainer_email='gom@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'bio_main = babo.main_integrated:main',      # 메인 오케스트레이터
        'bio_sub = babo.rack_transport_action:main',  # 하위 로봇 제어
        'bio_ui = babo.ui_integrated:main',
        ],
    },
)
