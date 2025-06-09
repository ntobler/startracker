import pathlib
import threading

import pytest

from startracker import camera, persistent, testing_utils, web_gui


def test_app_capture(tmp_path: pathlib.Path):
    persistent.Persistent._instance = None
    persistent.APPLICATION_USER_DIR = tmp_path
    assert persistent.Persistent.get_instance().user_data_dir == tmp_path

    cam = testing_utils.RandomStarCam
    cam.time_warp_factor = 1000
    cam.simulate_exposure_time = False
    cam.default_config = testing_utils.StarImageGeneratorConfig(
        exposure=200, catalog_max_magnitude=6.5
    )
    camera.RpiCamera = cam  # type: ignore[assignment, misc]

    app = web_gui.App()
    app_thread = threading.Thread(target=app.run)
    app_thread.start()
    try:
        app.get_state()

        app.capture(web_gui.CaptureMode.SINGLE)
        app.stream.get_blocking()
        with pytest.raises(TimeoutError):
            app.stream.get_blocking(timeout=1)

        app.capture(web_gui.CaptureMode.CONTINUOUS)
        app.stream.get_blocking()
        app.stream.get_blocking()
        app.capture(web_gui.CaptureMode.STOP)
        with pytest.raises(TimeoutError):
            app.stream.get_blocking(timeout=1)

    finally:
        app.terminate = True
        app_thread.join()


def test_app_auto_calibration(tmp_path: pathlib.Path):
    persistent.Persistent._instance = None
    persistent.APPLICATION_USER_DIR = tmp_path
    assert persistent.Persistent.get_instance().user_data_dir == tmp_path

    cam = testing_utils.StarCameraCalibrationTestCam
    cam.time_interval = 60
    cam.simulate_exposure_time = False
    cam.default_config = testing_utils.StarImageGeneratorConfig(
        exposure=200, catalog_max_magnitude=6.5
    )
    camera.RpiCamera = cam  # type: ignore[assignment, misc]

    app = web_gui.App()
    app_thread = threading.Thread(target=app.run)
    app_thread.start()
    try:
        app.capture(web_gui.CaptureMode.CONTINUOUS)
        app.auto_calibration("restart")
        app.stream.get_blocking()
        app.auto_calibration("discard")
        app.stream.get_blocking()
        app.auto_calibration("restart")

        for _ in range(20):
            s = app.stream.get_blocking()
            if s.get("auto_calibrator", {}).get("state") == "Improving calibration":
                break
        else:
            raise AssertionError  # pragma no cover

        assert not persistent.Persistent.get_instance().cam_file.is_file()

        app.auto_calibration("accept")

        assert persistent.Persistent.get_instance().cam_file.is_file()

        app.capture(web_gui.CaptureMode.STOP)
    finally:
        app.terminate = True
        app_thread.join()


if __name__ == "__main__":
    # import tempfile
    # with tempfile.TemporaryDirectory() as d:
    #     test_app_auto_calibration(pathlib.Path(d))
    pytest.main([__file__])
