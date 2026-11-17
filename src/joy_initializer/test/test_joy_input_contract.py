from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_input_node_has_no_force_feedback_api() -> None:
    source = (PACKAGE_ROOT / "src" / "joy_input_node.cpp").read_text(
        encoding="utf-8"
    )
    forbidden_symbols = (
        "SDL_INIT_HAPTIC",
        "SDL_Haptic",
        "JoyFeedback",
        "set_feedback",
        "GameControllerRumble",
        "HapticRumble",
    )

    for symbol in forbidden_symbols:
        assert symbol not in source


def test_input_node_uses_standard_game_controller_layout() -> None:
    source = (PACKAGE_ROOT / "src" / "joy_input_node.cpp").read_text(
        encoding="utf-8"
    )

    assert "SDL_INIT_GAMECONTROLLER" in source
    assert "SDL_GameControllerOpen" in source
    assert "SDL_CONTROLLER_AXIS_TRIGGERLEFT" in source
    assert "SDL_CONTROLLER_AXIS_TRIGGERRIGHT" in source
    assert "05000000c82d00001260000001000000" in source
    assert "lefttrigger:a5" in source
    assert "righttrigger:a4" in source
    assert "SDL_JoystickOpen" not in source


def test_launch_uses_project_bluetooth_input_node() -> None:
    launch_source = (PACKAGE_ROOT / "launch" / "joy.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'package="joy_initializer"' in launch_source
    assert 'executable="joy_input_node"' in launch_source
    assert 'package="joy"' not in launch_source
    assert 'device_name_contains",' in launch_source
    assert 'default_value="8BitDo"' in launch_source
    assert "8BitDo Ultimate 2 Wireless Controller for PC" not in launch_source
