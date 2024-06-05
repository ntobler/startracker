import pathlib
import tempfile

from startracker import protocol


def test_protocol():
    with tempfile.TemporaryDirectory() as d:
        file = pathlib.Path(d) / "proctocol.c"
        protocol.generate_code(file)
        assert file.exists()
