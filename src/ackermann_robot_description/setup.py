import os
from glob import glob
from setuptools import setup

package_name = "ackermann_robot_description"


def generate_paths(dir_name):
    # Get list of all file paths in given directory name (recursively)
    # The recursive approach gives all folder and file directories separately
    paths = glob(dir_name + "/**", recursive=True)

    # Filter list, to only keep file directories (no folder directories)
    paths = [path for path in paths if os.path.isfile(path)]

    # Create a sorted list of unique directories that contain files
    # Example: Files: models/gazebo/model.sdf, models/gazebo/model.config, models/turtle/model.sdf
    # becomes models/gazebo/, models/turtle/
    paths = sorted(list(set(["/".join(path.split("/")[0:-1]) for path in paths])))

    # Create a list of tuples (like in data_files) with (path, list_of_files)
    # glob takes any file (*.*) from the folder and adds it to the list, next to the according path in the tuple
    paths = [
        (os.path.join("share", package_name, path), glob(path + "/*.*"))
        for path in paths
    ]
    return paths


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*.*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.*")),
    ]
    + generate_paths(dir_name="models"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "display_odom = ackermann_robot_description.display_odom:main",
            "display_scan = ackermann_robot_description.display_scan:main",
            "display_imu = ackermann_robot_description.display_imu:main",
        ],
    },
)
