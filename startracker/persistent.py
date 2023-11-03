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
        self.data_dir = pathlib.Path("~/.startracker").expanduser().absolute()
        self.data_dir.mkdir(exist_ok=True)

        self.repo_dir = pathlib.Path(__file__).parent.absolute().parent

        self.calibration_dir = self.data_dir
        self.calibration_dir.mkdir(exist_ok=True)
