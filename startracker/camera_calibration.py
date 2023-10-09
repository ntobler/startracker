"""Camera calibration tool."""

import matplotlib.pyplot as plt
import matplotlib
from matplotlib.widgets import Button, Slider
import numpy as np
import logging

from camera import Camera
import image_processing

class CameraGui:
    def __init__(self, cam: Camera):
        self._cam = cam

        fig = plt.figure()

        self._image_ax = fig.add_axes([0.05, 0.2, 0.9, 0.75])  # left bottom width height
        self._image_ax.imshow(np.random.normal(size=(200, 300)))

        ax_capture = fig.add_axes([0.81, 0.05, 0.1, 0.075])
        button_capture = Button(ax_capture, "Capture")
        button_capture.on_clicked(self._capture)

        ax_exposure = fig.add_axes([0.25, 0.1, 0.65, 0.03])
        slider_exposure = Slider(
            ax=ax_exposure, label="Exposure", valmin=0.1, valmax=100, valinit=0.1
        )
        slider_exposure.on_changed(self._set_exposure)

        plt.show()

    def _capture(self, mouse_event):
        print("capture")
        image = self._cam.capture_raw()[:, :-16]
        #TODO correct black level image -= 250
        image = image_processing.binning(image, factor=4)
        # TODO use .set_data(image) instead
        self._image_ax.imshow(image)
        plt.draw()

    def _set_exposure(self, exposure: float):
        self._cam.exposure_ms = exposure
        print(f"set exposure: {exposure}")

    def show(self):
        pass
        # plt.show()


def main() -> int:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
        handlers=[logging.StreamHandler()],
    )

    matplotlib.rcParams["webagg.open_in_browser"] = False
    matplotlib.rcParams["webagg.address"] = "0.0.0.0"
    matplotlib.use("WebAgg")

    cam = Camera(exposure_ms=1)

    with cam:
        try:
            CameraGui(cam).show()
        except KeyboardInterrupt:
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
