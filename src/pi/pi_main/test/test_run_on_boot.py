from pathlib import Path

from pi_main.run_on_boot import main


def test_install_on_boot() -> None:
    """Test that file copying and systemd are made."""
    # TODO: update to clean up files
    return

    main()

    # Test for files being copied correctly
    actual_rules_files = set((Path('/etc') / 'udev' / 'rules.d').iterdir())
    expected_rules_files = {'i2c.rules', 'camera.rules', 'pixhawk.rules'}
    assert expected_rules_files.issubset(actual_rules_files)

    assert (Path('/etc') / 'systemd' / 'system' / 'pi_main.service').exists()
