from setuptools import setup

package_name = 'ball_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu2004',
    maintainer_email='teturou@aw.wakwak.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'show_image_node = ball_tracking.show_image:main', 
            'ball_tracking_node = ball_tracking.ball_tracking:main'
        ],
    },
)
