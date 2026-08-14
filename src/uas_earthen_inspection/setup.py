from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'uas_earthen_inspection'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Person 1',
    maintainer_email='person1@earthen-uav.org',
    description='Zero-Shot VLM RAG Defect Inspection & Revisit Loop Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'per_waypoint_capture_node = uas_earthen_inspection.per_waypoint_capture_node:main',
            'detection_node = uas_earthen_inspection.detection_node:main',
            'rag_knowledge_base = uas_earthen_inspection.rag_knowledge_base:main',
            'revisit_waypoint_generator = uas_earthen_inspection.revisit_waypoint_generator:main',
        ],
    },
)
