from pathlib import Path
from xml.etree import ElementTree


ROOT = Path(__file__).resolve().parents[1]


def test_package_metadata_has_runtime_dependencies_and_no_placeholders():
    root = ElementTree.parse(ROOT / "package.xml").getroot()
    description = root.findtext("description")
    license_text = root.findtext("license")
    exec_depends = {element.text for element in root.findall("exec_depend")}

    assert description and "TODO" not in description
    assert license_text and "TODO" not in license_text
    assert {
        "control_msgs",
        "mediapipe_ros2_interfaces",
        "mediapipe_ros2_py",
        "controller_manager",
        "joint_trajectory_controller",
        "usb_cam",
    } <= exec_depends


def test_launch_files_expose_same_actions_topics_and_hand_selection():
    real = (ROOT / "launch" / "real.launch.py").read_text()
    sim = (ROOT / "launch" / "sim.launch.py").read_text()

    for source in (real, sim):
        assert 'choices=["right", "left", "both"]' in source
        assert "/right_hand_controller/follow_joint_trajectory" in source
        assert "/left_hand_controller/follow_joint_trajectory" in source
        assert "/mediapipe" in source
        assert "/inspire_hand/target_joint_states" in source


def test_install_contract_includes_runtime_assets_and_scripts():
    cmake = (ROOT / "CMakeLists.txt").read_text()
    assert "ament_python_install_package" in cmake
    assert "scripts/inspire_hand_teleop" in cmake
    assert "scripts/inspire_hand_command" in cmake
    assert "scripts/video_file_image_publisher" in cmake
    for directory in ("config", "description", "launch", "rviz", "test_data"):
        assert directory in cmake


def test_vendored_dex_urdf_assets_are_present_with_license():
    assert (ROOT / "description" / "inspire_hand" / "LICENSE.txt").exists()
    assert (ROOT / "description" / "inspire_hand" / "inspire_hand_right.urdf").exists()
    assert (ROOT / "description" / "inspire_hand" / "inspire_hand_left.urdf").exists()
    assert (ROOT / "description" / "inspire_hand" / "meshes" / "visual").is_dir()
