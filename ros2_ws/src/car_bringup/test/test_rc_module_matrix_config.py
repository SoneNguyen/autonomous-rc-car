from pathlib import Path


def test_rc_module_matrix_config_is_covered_by_recursive_install():
    package_root = Path(__file__).resolve().parents[1]
    cmake = (package_root / "CMakeLists.txt").read_text(encoding="utf-8")
    assert 'file(GLOB_RECURSE CAR_BRINGUP_CONFIG_FILES CONFIGURE_DEPENDS' in cmake
    assert 'install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)' in cmake
    config = package_root / "config/rc_module_matrix"
    assert config.is_dir(), "Module-matrix config directory is missing"


def test_module_matrix_config_contains_expected_case_files():
    package_root = Path(__file__).resolve().parents[1]
    config = package_root / "config/rc_module_matrix"
    paths = sorted(config.glob("*.yaml"))
    assert paths, "Module scenario configs are missing"
    assert {path.stem for path in paths} <= {
        "lidar", "camera", "imu_odometry", "contact_tf", "command_path"
    }
