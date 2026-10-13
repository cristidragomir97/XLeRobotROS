from glob import glob
from setuptools import setup

setup(name='xlerobot_sim', version='0.1.0', packages=['xlerobot_sim'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/xlerobot_sim']),
                  ('share/xlerobot_sim', ['package.xml']),
                  ('share/xlerobot_sim/launch', glob('launch/*.launch.py')),
                  ('share/xlerobot_sim/config', glob('config/*')),
                  ('share/xlerobot_sim/worlds', glob('worlds/*.xml'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Robocore', maintainer_email='dev@robocore.ai',
      description='XLeRobot simulation and RGB-D SLAM', license='Apache-2.0',
      entry_points={'console_scripts': ['sim = xlerobot_sim.node:main',
                                        'viewer = xlerobot_sim.viewer:main',
                                        'camera = xlerobot_sim.camera:main']})
