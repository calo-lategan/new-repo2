import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'app'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'resource'), glob(os.path.join('resource', '*.dae'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='1270161395@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration = app.calibration:main',
            'object_tracking = app.object_tracking:main',
            'color_tracker = app.color_tracker:main',
            # 'finger_trace = app.finger_trace:main',
            'object_sorting = app.object_sorting:main',
            'custom_sorting = app.custom_sorting:main',
            'tag_stackup = app.tag_stackup:main',
            'shape_recognition = app.shape_recognition:main',
            'waste_classification = app.waste_classification:main',
            'finger_trace = app.finger_trace:main',
            'lab_manager = app.lab_manager:main',
            'custom_sortingv4 = app.custom_sortingv4:main',
            'tune_uiv4 = app.tune_uiv4:main',
            'custom_sortingv4_1 = app.custom_sortingv4_1:main',
            'tune_uiv4_1 = app.tune_uiv4_1:main',
            'custom_sortingv5 = app.custom_sortingv5:main',
            'tune_uiv5 = app.tune_uiv5:main',
            'custom_sortingv51 = app.custom_sortingv51:main',
            'tune_uiv51 = app.tune_uiv51:main',
        ],
    },
)
