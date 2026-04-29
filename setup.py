from setuptools import find_packages, setup
import os
from glob import glob

package_name = "sensor_fusion_pkg"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        #registering the package with ament's resource index
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        #installing package.xml so ROS2 tooling can find it
        (f"share/{package_name}", ["package.xml"]),
        #install launch files so `ros2 launch` can find them
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kiran Sairam",
    maintainer_email="kiransairam1@gmail.com.com",
    description=(
        "Fuses IMU and depth sensor data via a complementary filter "
        "to estimate vertical velocity."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            #the format is usually "executable_name = package_name.module_name:function_name"
            #this is what `ros2 run sensor_fusion_pkg fused_data` calls.
            "fused_data = sensor_fusion_pkg.fused_data_node:main",
        ],
    },
)
