from setuptools import find_packages, setup


package_name = "car_decision"

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
    description="Obstacle course multi-sensor evidence and attributed obstacle decisions",
    license="MIT",
    entry_points={
        "console_scripts": [
            "lidar_evidence_node = car_decision.lidar_evidence_node:main",
            "camera_evidence_node = car_decision.camera_evidence_node:main",
            "imu_evidence_node = car_decision.imu_evidence_node:main",
            "odometry_evidence_node = car_decision.odometry_evidence_node:main",
            "contact_evidence_node = car_decision.contact_evidence_node:main",
            "decision_node = car_decision.decision_node:main",
            "planner_intent_adapter = car_decision.planner_intent_adapter:main",
            "controller_request_adapter = car_decision.controller_request_adapter:main",
            "vehicle_visual_overlay = car_decision.vehicle_visual_overlay:main",
            "manual_twist_adapter = car_decision.manual_twist_adapter:main",
            "fault_gate_node = car_decision.fault_gate_node:main",
            "run_logger = car_decision.run_logger:main",
            "dashboard_node = car_decision.dashboard_node:main",
        ],
    },
)
