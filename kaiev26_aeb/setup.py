from glob import glob
from setuptools import setup

setup(
    name='kaiev26_aeb', version='0.1.0', packages=['kaiev26_aeb'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/kaiev26_aeb']),
        ('share/kaiev26_aeb', ['package.xml', 'README.md']),
        ('share/kaiev26_aeb/config', glob('config/*')),
        ('share/kaiev26_aeb/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
    maintainer='sj', maintainer_email='user@example.com', license='Proprietary',
    description='Cone-course path generation and Pure Pursuit or Stanley tracking.',
    entry_points={'console_scripts': [
        'cone_colored_path_node = kaiev26_aeb.colored_path_node:main',
        'cone_pursuit_node = kaiev26_aeb.tracking_node:main_pursuit',
        'cone_stanley_node = kaiev26_aeb.tracking_node:main_stanley',
        'cone_trial_tui = kaiev26_aeb.trial_tui:cli',
    ]},
)
