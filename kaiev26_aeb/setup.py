from glob import glob
from setuptools import setup

setup(
    name='kaiev26_aeb', version='0.1.0', packages=['kaiev26_aeb'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/kaiev26_aeb']),
        ('share/kaiev26_aeb', ['package.xml', 'README.md', 'requirements-offline.txt']),
        ('share/kaiev26_aeb/config', glob('config/*')),
        ('share/kaiev26_aeb/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
    maintainer='sj', maintainer_email='user@example.com', license='Proprietary',
    description='Fusion or camera-only cone paths with Pure Pursuit and Stanley tracking.',
    entry_points={'console_scripts': [
        'cone_path_node = kaiev26_aeb.perception_node:main_fusion',
        'cone_yolo_path_node = kaiev26_aeb.perception_node:main_yolo',
        'process_bag = kaiev26_aeb.bag:main',
        'process_yolo_bag = kaiev26_aeb.yolo_bag:main',
        'cone_pursuit_node = kaiev26_aeb.tracking_node:main_pursuit',
        'cone_stanley_node = kaiev26_aeb.tracking_node:main_stanley',
        'check_tracking_bag = kaiev26_aeb.check_tracking_bag:main',
        'sim_trial = kaiev26_aeb.sim_trial:main',
        'cone_trial_tui = kaiev26_aeb.trial_tui:cli',
    ]},
)
