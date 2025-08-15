from setuptools import setup
package_name = 'raspberry_perception'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/perception_pipeline.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='ROI selection, SAM mask, manager orchestration for FoundationPose',
    license='MIT',
    entry_points={
        'console_scripts': [
            'manual_selection_node = raspberry_perception.manual_selection_node:main',
            'sam_segmentation_node = raspberry_perception.sam_segmentation_node:main',
            'manager_node = raspberry_perception.manager_node:main',
        ],
    },
)
