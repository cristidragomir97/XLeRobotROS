from setuptools import find_packages, setup
from glob import glob

package_name = 'robocore_viz'

setup(
    name=package_name,
    version='1.3.1',
    packages=find_packages(exclude=['test']),
    package_data={
        'robocore_viz': [
            'html/*',
            'html/css/*',
            'html/css/images/*',
            'html/fonts/*',
            'html/js/*',
            'html/js/transports/*',
            'html/js/viewers/*',
            'html/js/viewers/meta/*',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'LICENSE', 'PROVENANCE.md', 'README.md']),
        ('share/' + package_name + '/examples', glob('examples/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Robocore Labs',
    maintainer_email='dheera@dheera.net',
    description='Configurable robot dashboard and ROS websocket bridge',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robocore_viz_node = robocore_viz.robocore_viz:main',
        ],
    },
)
