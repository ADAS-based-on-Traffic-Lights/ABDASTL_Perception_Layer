from setuptools import setup

package_name = 'tl_perception'

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
    maintainer='nvidia',
    maintainer_email='nvidia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'model_inference = tl_perception.model_inference:main',
                'depth_estimation = tl_perception.depth_estimation:main',
                'tl_decision_making = tl_perception.tl_decision_making:main'
        ],
    },
)
