from setuptools import find_packages, setup

package_name = "car_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Son Nguyen",
    maintainer_email="174254834+SoneNguyen@users.noreply.github.com",
    description="RC sensor-frame transforms",
    license="MIT",
    entry_points={
        "console_scripts": [
            "sensor_tf_node = car_navigation.sensor_tf_node:main",
        ],
    },
)
