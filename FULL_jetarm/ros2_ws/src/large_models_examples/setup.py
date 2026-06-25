import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'large_models_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'large_models_examples'), glob(os.path.join('large_models_examples', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='1270161395@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_transport = large_models_examples.object_transport:main',
            'intelligent_grasp = large_models_examples.intelligent_grasp:main',
            'llm_control_servo = large_models_examples.llm_control_servo:main',
            'llm_object_sorting = large_models_examples.llm_object_sorting:main',
            'llm_waste_classification = large_models_examples.llm_waste_classification:main',
            'llm_3d_object_sorting = large_models_examples.llm_3d_object_sorting:main',
            'vllm_with_camera = large_models_examples.vllm_with_camera:main',
            'vllm_track = large_models_examples.vllm_track:main',
            'vllm_dietitianl = large_models_examples.vllm_dietitianl:main',
            'vllm_object_transport = large_models_examples.vllm_object_transport:main',
        ],
    },
)
