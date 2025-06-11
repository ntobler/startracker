import pathlib
import threading

import numpy as np
import pytest

from startracker import camera, persistent, testing_utils, web_gui


def test_image_encoder():
    rng = np.random.default_rng(42)
    img = rng.integers(0, 20, size=(540, 960), dtype=np.uint8)

    size_kb = 1024
    ie = web_gui.ImageEncoder(max_kb=size_kb / 1024)

    for size_kb in [40, 200, None, 100, 50, 150, 30]:
        ie.max_kb = size_kb
        loop_broken = False
        for _ in range(15):
            encoded = ie.encode(img)
            assert encoded is not None
            if size_kb is None:
                if ie.quality_str == "PNG":
                    loop_broken = True
                    break
            else:
                if np.abs(len(encoded) / 1024 - size_kb) < size_kb / 10:
                    loop_broken = True
                    break
        assert loop_broken


def test_web_gui_start(tmp_path: pathlib.Path):
    testing_utils.patch_persistent(tmp_path, cal=True)

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
    finally:
        app.terminate = True
        app_thread.join()


def test_app_capture(tmp_path: pathlib.Path):
    testing_utils.patch_persistent(tmp_path, cal=False)

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
    testing_utils.patch_persistent(tmp_path, cal=False)

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

        loop_broken = False
        for _ in range(20):
            s = app.stream.get_blocking()
            if s.get("auto_calibrator", {}).get("state") == "Improving calibration":
                loop_broken = True
                break
        assert loop_broken

        assert not persistent.Persistent.get_instance().cam_file.is_file()

        app.auto_calibration("accept")

        assert persistent.Persistent.get_instance().cam_file.is_file()

        app.capture(web_gui.CaptureMode.STOP)
    finally:
        app.terminate = True
        app_thread.join()


def test_axis_calibration(tmp_path: pathlib.Path):
    persistent.Persistent._instance = None
    testing_utils.patch_persistent(tmp_path, cal=True)

    cam = testing_utils.AxisAlignCalibrationTestCam
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
        app.stream.get_blocking()
        assert app.axis_calibration("put")["axis_calibration"]["calibration_orientations"] == 1
        assert app.axis_calibration("put")["axis_calibration"]["calibration_orientations"] == 2
        assert app.axis_calibration("reset")["axis_calibration"]["calibration_orientations"] == 0
        for i in range(1, 5):
            app.stream.get_blocking()
            assert app.axis_calibration("put")["axis_calibration"]["calibration_orientations"] == i
        assert not persistent.Persistent.get_instance().axis_rot_quat_file.is_file()
        assert app.axis_calibration("calibrate")["axis_calibration"]["calibration_error_deg"] < 1.0
        assert persistent.Persistent.get_instance().axis_rot_quat_file.is_file()
        app.capture(web_gui.CaptureMode.STOP)
    finally:
        app.terminate = True
        app_thread.join()


if __name__ == "__main__":
    pytest.main([__file__])
