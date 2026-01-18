from setuptools import find_packages, setup

package_name = 'bio_transport'

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
        'rack_transport = bio_transport.rack_transport_node:main',          # 파라미터 실행형
        'rack_transport_server = bio_transport.rack_transport_server:main', # UI/토픽 대기형(신규 파일)
        'rack_pick = bio_transport.rack_pick_node:main',
        'bio_test = bio_transport.test_node:main',
        'rack_transport_ui = bio_transport.rack_transport_ui:main',
        'rack_inbound = bio_transport.rack_inbound_node:main',
        'rack_outbound = bio_transport.rack_outbound_node:main',
        ],
    },
)
