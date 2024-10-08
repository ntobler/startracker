"""Persistent data storage module."""

import pathlib


class Persistent:
    """Class to handle persistent file storage."""

    _instance = None

    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def __init__(self):
        """Create the application directory `~/.startracker` if not already present."""
        self.user_data_dir = pathlib.Path("~/.startracker").expanduser().absolute()
        self.user_data_dir.mkdir(exist_ok=True)

        self.repo_dir = pathlib.Path(__file__).parent.absolute().parent

        self.calibration_dir = self.user_data_dir / "calibration"
        self.calibration_dir.mkdir(exist_ok=True)

        self.conf_file = self.user_data_dir / "config.yml"

        self.package_dir = pathlib.Path(__file__).parent.expanduser().absolute()

        self.default_conf_file = self.package_dir / "default_config.yml"

        self.cam_file = self.user_data_dir / "cam_file.json"

        self.dark_frame_file = self.user_data_dir / "darkframe.npy"

        self.cam_config_file = self.user_data_dir / "cam_config.pkl"

        self.axis_rot_quat_file = self.user_data_dir / "axis_rot_quat.npy"
