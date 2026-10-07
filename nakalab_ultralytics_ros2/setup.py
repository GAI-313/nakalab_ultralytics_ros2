import os
from setuptools import find_packages, setup

package_name = 'nakalab_ultralytics_ros2'
data_files = []
data_files.append(
    ("share/ament_index/resource_index/packages", ["resource/" + package_name])
)
data_files.append(("share/" + package_name, ["package.xml"]))


def package_files(directory, data_files):
    for path, directories, filenames in os.walk(directory):
        directories[:] = sorted(d for d in directories if not d.startswith(('.', '__pycache__')))
        files = [os.path.join(path, name) for name in sorted(filenames)
                 if not name.startswith('.') and not name.endswith('.pyc')]
        if files:
            data_files.append(("share/" + package_name + "/" + path, files))
    return data_files


# Add directories
data_files = package_files("launch", data_files)
data_files = package_files("models", data_files)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roboworks',
    maintainer_email='roboworks@katana-2024-4',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'person_pose = nakalab_ultralytics_ros2.person_pose:main',
            'object_seg_pose = nakalab_ultralytics_ros2.object_seg_pose:main',
        ],
    },
)
