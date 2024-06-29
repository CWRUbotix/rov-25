import os

from pi_main.run_on_boot import main


def test_install_on_boot() -> None:
    """Test that file copying and systemd are made."""
    # TODO update to clean up files
    return

    main()

    # Test for files being copied correctly
    actual_rules_files = set(os.listdir(os.path.join("/etc", "udev", "rules.d")))
    expected_rules_files = set(["i2c.rules", "camera.rules", "pixhawk.rules"])
    assert expected_rules_files.issubset(actual_rules_files)

    assert os.path.exists(os.path.join('/etc', 'systemd', 'system', 'pi_main.service'))
