from glob import glob
from setuptools import find_packages, setup


package_name = "hl_ku_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/routes", glob("routes/*.csv")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    extras_require={
        "smoothing": ["numpy>=1.21", "scipy>=1.8"],
        "test": ["pytest", "numpy>=1.21", "scipy>=1.8", "PyYAML"],
    },
    zip_safe=True,
    maintainer="HL KU Team",
    maintainer_email="hl-ku@example.com",
    description="GNSS-first autonomous driving foundation for HL FMA 1/5.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "um982_serial = hl_ku_core.um982_serial_node:main",
            "ntrip_client = hl_ku_core.ntrip_client_node:main",
            "gnss_localizer = hl_ku_core.gnss_localizer_node:main",
            "velocity_selector = hl_ku_core.velocity_selector_node:main",
            "camera_perception = hl_ku_core.camera_perception_node:main",
            "lidar_perception = hl_ku_core.lidar_perception_node:main",
            "perception_aggregator = hl_ku_core.perception_aggregator_node:main",
            "path_tracker = hl_ku_core.path_tracker_node:main",
            "mission_manager = hl_ku_core.mission_manager_node:main",
            "vehicle_controller = hl_ku_core.vehicle_controller_node:main",
            "safety_supervisor = hl_ku_core.safety_supervisor_node:main",
            "nucleo_serial_bridge = hl_ku_core.nucleo_serial_bridge_node:main",
            "gnss_survey = hl_ku_core.gnss_survey_node:main",
            "route_recorder = hl_ku_core.route_recorder_node:main",
            "route_prepare = hl_ku_core.route_prepare:main",
            "rtk_waypoint_recorder = hl_ku_core.rtk_waypoint_recorder_node:main",
            "preflight_check = hl_ku_core.preflight_check_node:main",
        ]
    },
)
