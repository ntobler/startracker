"""Various helper function and classes for testing, especially for star and camera simulation."""

import abc
import dataclasses
import pathlib
import queue
import time
from typing import Optional

import cv2
import numpy as np
import numpy.typing as npt
import ruststartracker
import scipy.spatial.transform
import serial
from typing_extensions import Buffer, override

from startracker import (
    calibration,
    camera,
    const,
    image_utils,
    kalkam,
    persistent,
    util,
)

UINT8_MAX = 255


def patch_persistent(path: pathlib.Path, *, cal: bool):
    """Set persistent data directory with a new folder."""
    path = pathlib.Path(path)
    if not path.is_dir():
        msg = f"Path {path} must be an existing directory"
        raise ValueError(msg)
    persistent.Persistent._instance = None
    persistent.APPLICATION_USER_DIR = path
    p = persistent.Persistent.get_instance()
    if cal:
        calibration.make_dummy().to_json(p.cam_file)


class MockSerial(serial.SerialBase):
    def __init__(self):
        super().__init__()
        self.mosi_channel = queue.Queue()
        self.miso_channel = queue.Queue()

    @override
    def read(self, size: int = -1, /) -> bytes:
        ret = bytes(self.mosi_channel.get() for _ in range(size))
        print(f"Device read: {ret!r}")
        return ret

    @override
    def write(self, data: Buffer) -> int:
        data = bytes(data)
        print(f"Device wrote: {data!r}")
        for x in data:
            self.miso_channel.put(x)
        return len(data)

    def reset_input_buffer(self) -> None:
        """Clear the input buffer."""
        while not self.mosi_channel.empty():
            self.mosi_channel.get_nowait()


class MockSerialMaster(serial.SerialBase):
    def __init__(self, test_serial: MockSerial):
        super().__init__()
        self.test_serial = test_serial

    @override
    def read(self, size: int = -1, /) -> bytes:
        ret = bytes(self.test_serial.miso_channel.get() for _ in range(size))
        print(f"Device read: {ret!r}")
        return ret

    @override
    def write(self, data: Buffer) -> int:
        data = bytes(data)
        print(f"Master wrote: {data!r}")
        for x in data:
            self.test_serial.mosi_channel.put(x)
        return len(data)

    def reset_input_buffer(self):
        """Clear the input buffer."""
        while not self.test_serial.miso_channel.empty():
            self.test_serial.miso_channel.get_nowait()


@dataclasses.dataclass
class StarImageGeneratorConfig:
    """Configuration parameters for the StarImageGenerator."""

    exposure: float = 15.0
    """Simulated exposure time in seconds. Higher exposure results in brighter image."""
    blur: float = 0.7
    """Blur post-processing standard deviation."""
    noise_sigma: float = 1.0
    """Added noise standard deviation."""
    black_level: float = 4.5
    """Black level of sensor."""
    catalog_max_magnitude: float = 5.5
    """Maximum star intensity included in the star catalog. Higher number is more faint."""


class StarImageGenerator:
    distorter: Optional[kalkam.PointUndistorter]

    def __init__(
        self, cal: kalkam.IntrinsicCalibration, config: Optional[StarImageGeneratorConfig] = None
    ):
        """Initialize."""
        if config is None:
            config = StarImageGeneratorConfig()

        self._cal = cal
        self.width, self.height = cal.image_size
        self.intrinsic = cal.intrinsic

        self.exposure = config.exposure
        self.blur = config.blur
        self.noise_sigma = config.noise_sigma
        self.black_level = config.black_level

        self._cos_phi = self._cal.cos_phi(angle_margin_factor=1.2)

        if self._cal.dist_coeffs is not None:
            self.distorter = kalkam.PointUndistorter(self._cal)
        else:
            self.distorter = None

        catalog = ruststartracker.StarCatalog(max_magnitude=config.catalog_max_magnitude)
        self.stars_nwu = catalog.normalized_positions()
        self.stars_mags = catalog.magnitude

        self._rng = np.random.default_rng(42)

        self._offset = np.array(
            [
                [[0, 0], [0, 1]],
                [[1, 0], [1, 1]],
            ],
            dtype=np.uint32,
        )

    def __call__(
        self,
        target_vector: npt.ArrayLike,
        up_vector: npt.ArrayLike,
        *,
        grid: bool = False,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Create image of stars.

        Args:
            target_vector: shape=[3] Equatorical target vector, the camera points at in astronomical
                coordinate system e.g. north=[0,0,1]
            up_vector: shape=[3] Up-facing vector of the camera in astronomical coordinate system
                e.g. north=[0,0,1]
            grid: Display longitude and latitude overlay

        Returns:
            tuple[np.ndarray, np.ndarray, np.ndarray]:
                - Grayscale uint8 star image
                - Painted star coordinates in pixels shape = [n, 2 (x, y)]
                - Painted star magnitudes
        """
        # Build extrinsic
        target_vector = np.array(target_vector, np.float32)
        target_vector /= np.linalg.norm(target_vector, axis=0)
        up_vector = np.array(up_vector, np.float32)
        up_vector /= np.linalg.norm(up_vector, axis=0)
        extrinsic = kalkam.look_at_extrinsic(target_vector, [0, 0, 0], up_vector)

        return self.image_from_extrinsic(extrinsic, grid=grid)

    def image_from_quaternion(
        self, quat: np.ndarray, *, grid: bool = False
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Create image of stars from a quaternion."""
        rot = scipy.spatial.transform.Rotation.from_quat(quat)
        extrinsic = np.concatenate((rot.as_matrix().T, np.zeros((3, 1))), axis=-1)
        return self.image_from_extrinsic(extrinsic, grid=grid)

    def stars_from_extrinsic(self, extrinsic: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Get star magnitudes and pixel positions from extrinsic matrix."""
        # Take z image vector of the inverted extrinsic
        target_vector = extrinsic[2, :3]

        # Select all stars that are roughly in frame
        in_frame = np.inner(self.stars_nwu, target_vector) > self._cal.cos_phi()
        stars_nwu = self.stars_nwu[in_frame]
        stars_mags = self.stars_mags[in_frame]

        # Convert stars to pixels
        pp = kalkam.PointProjector(self._cal, extrinsic)
        stars_xy = pp.obj2pix(stars_nwu, axis=-1)

        return stars_mags, stars_xy

    def image_from_extrinsic(
        self, extrinsic: np.ndarray, *, grid: bool = False
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Create image of stars from extrinsic matrix."""
        width, height = self.width, self.height

        stars_mags, stars_xy = self.stars_from_extrinsic(extrinsic)

        if False:
            import matplotlib.pyplot as plt

            plt.plot(stars_xy[..., 0], stars_xy[..., 1], ".")
            w, h = width, height
            plt.plot([0, w, w, 0, 0, 0], [0, 0, h, h, 0, 0])
            plt.axis("equal")
            plt.show()

        # Make pixel quads that distribute the brightness in sub-pixel accuracy
        pixel_coords = stars_xy[..., None, None, :].astype(np.int32)
        pixel_coords = pixel_coords + self._offset
        pixel_coords = pixel_coords.reshape((-1, 2))
        pixel_remainder = np.mod(stars_xy, 1)

        # Convert star magnitude (visible flux) to a pixel value (may be way above 255)
        star_brightness = (100 ** (1 / 5)) ** (-stars_mags)
        star_brightness = star_brightness * (self.exposure * UINT8_MAX)

        # Distribute the brightness to each pixel of the quad according to the distance
        pixel_mags = np.broadcast_to(star_brightness[..., None, None], (len(stars_xy), 2, 2)).copy()
        pixel_mags[..., ::2, :] *= 1 - pixel_remainder[..., None, None, 0]
        pixel_mags[..., 1::2, :] *= pixel_remainder[..., None, None, 0]
        pixel_mags[..., :, ::2] *= 1 - pixel_remainder[..., None, None, 1]
        pixel_mags[..., :, 1::2] *= pixel_remainder[..., None, None, 1]
        pixel_mags = pixel_mags.ravel()

        # Mask pixels that are in frame, now accurately
        mask = np.logical_and(pixel_coords >= 0, pixel_coords < (width, height)).all(axis=-1)
        pixel_mags = pixel_mags[mask]
        pixel_coords = pixel_coords[mask]

        # Paint quad pixel sized stars on floating point canvas
        canvas = np.zeros((height, width), np.float32)
        x, y = pixel_coords.T
        canvas[y, x] = pixel_mags

        # Blur canvas to make very bright stars wider
        if self.blur is not None:
            # set kernel size to 4 sigmas
            kernel_size = np.minimum(self.blur * 4, 25)
            # Round up to next odd integer
            kernel_size = int(np.ceil(kernel_size) // 2 * 2 + 1)
            canvas_ = cv2.GaussianBlur(canvas, (kernel_size, kernel_size), self.blur)
            canvas = np.asarray(canvas_, dtype=np.float32)  # for typing
        if self.noise_sigma is not None:
            noise = self._rng.standard_normal(size=canvas.shape, dtype=np.float32)
            noise *= self.noise_sigma
            noise += self.black_level
            canvas += noise

        # Clip image and convert image to uint8
        np.clip(canvas, 0, UINT8_MAX, out=canvas)
        canvas = canvas.astype(np.uint8)

        # Mask vectors to contain only visible stars
        mask = np.logical_and(stars_xy >= 0, stars_xy < (width, height)).all(axis=-1)
        stars_xy = stars_xy[mask]
        stars_mags = stars_mags[mask]

        if grid:
            image_utils.draw_grid(canvas, extrinsic, self._cal, inplace=True)

        return canvas, stars_xy, stars_mags


class CameraTester:
    """Show matplotlib GUI to play with a debug camera."""

    @dataclasses.dataclass
    class Slider:
        """Slider configuration."""

        name: str
        min: float
        max: float
        init: float
        step: float
        dtype: type[float | int]

    sliders: list[Slider]

    def __init__(self, cam: camera.Camera):
        self._cam = cam
        self.sliders = []

    def add_arg(
        self,
        name: str,
        min: float = 0,
        max: float = 100,
        init: Optional[float] = None,
        step: Optional[float] = None,
        dtype: type[float | int] = int,
    ):
        """Add slider for a numerical attribute of the camera."""
        init = (min + max) / 2 if init is None else init
        step = (max - min) / 100 if step is None else step
        self.sliders.append(self.Slider(name, min, max, init, step, dtype))

    def run(self):
        """Run matplotlib GUI."""
        import matplotlib.pyplot as plt
        import matplotlib.widgets

        # Create a figure and axes
        fig, ax = plt.subplots()
        plt.subplots_adjust(bottom=0.25)

        # Generate initial image
        for slider in self.sliders:
            value = slider.dtype(slider.init)
            cam.__setattr__(slider.name, value)
        initial_image = cam.capture()
        img = ax.imshow(initial_image, cmap="viridis", vmax=255)

        plt_sliders = []

        # Callback function to update the image
        def update(val):
            _ = val
            for slider, plt_slider in zip(self.sliders, plt_sliders):
                value = slider.dtype(plt_slider.val)
                cam.__setattr__(slider.name, value)

            updated_image = cam.capture()

            img.set_data(updated_image)
            plt.draw()

        # Add two horizontal sliders
        for i, slider in enumerate(self.sliders):
            pos = 0.1 * (i + 1) / len(self.sliders)
            ax = plt.axes((0.1, pos, 0.65, 0.03), facecolor="lightgoldenrodyellow")
            plt_slider = matplotlib.widgets.Slider(
                ax,
                slider.name,
                slider.min,
                slider.max,
                valinit=slider.init,
                valstep=slider.step,
            )
            plt_slider.on_changed(update)
            plt_sliders.append(plt_slider)

        plt.show()


class ArtificialStarCam(camera.Camera):
    """Camera to generate artificial images of the sky."""

    cal: kalkam.IntrinsicCalibration
    """Calibration used to create mock images"""
    t: Optional[float] = None
    """Capture time in seconds."""
    time_interval: Optional[float] = None
    """Time interval between capture times. Ignored if t is set."""
    time_warp_factor: float = 1.0
    """If t is not given, accelerate time by this factor. Ignored if time_interval is set."""
    grid: bool = False
    """Display longitude and latitude grid overlay."""
    simulate_exposure_time: bool = False
    """Wait until exposure time has elapsed."""
    default_config: StarImageGeneratorConfig = StarImageGeneratorConfig()
    """Default configuration of the star image generator if None given.
    Set this class variable to use config for newly instantiated objects."""

    def __init__(
        self,
        camera_settings: camera.CameraSettings,
        cal: Optional[kalkam.IntrinsicCalibration] = None,
        config: Optional[StarImageGeneratorConfig] = None,
    ):
        super().__init__(camera_settings)
        self._rng = np.random.default_rng(42)
        self.cal = calibration.make_dummy() if cal is None else cal
        config = config if config is not None else self.default_config
        self._sig = StarImageGenerator(self.cal, config)
        self._last_star_image_positions: Optional[npt.NDArray[np.float64]] = None

    @override
    def capture_raw(self) -> npt.NDArray[np.uint16]:
        return self.capture().astype(np.uint16)

    @override
    def capture(self) -> npt.NDArray[np.uint8]:
        if self.t is not None:
            self.capture_time = self.t
        elif self.time_interval is not None:
            self.capture_time += self.time_interval
        else:
            self.capture_time = time.monotonic() * self.time_warp_factor

        if self.simulate_exposure_time:
            with util.max_rate(1000 / self._sig.exposure):
                image, self._last_star_image_positions = self._capture()
                return image
        else:
            image, self._last_star_image_positions = self._capture()
            return image

    def get_last_star_image_positions(self) -> npt.NDArray[np.float64]:
        """Get image positions of stars contained in previously captured image."""
        if self._last_star_image_positions is None:
            msg = "No image captured so far"
            raise ValueError(msg)
        return self._last_star_image_positions

    @override
    def record_darkframe(self) -> None:
        pass

    @abc.abstractmethod
    def _capture(self) -> tuple[npt.NDArray[np.uint8], npt.NDArray[np.float64]]:
        """Return captured image and star image positions."""

    @override
    def _apply_settings(self) -> None:
        self._sig.exposure = self._settings.exposure_ms * 15 / 100

    def gui(self) -> None:
        """Play with this camera in a GUI."""
        ct = CameraTester(self)
        ct.add_arg("t", 0, 60 * 60 * 24, init=0, dtype=float)
        ct.run()


class RandomStarCam(ArtificialStarCam):
    """Star camera pointing in a random direction and wiggeling."""

    def __init__(
        self,
        camera_settings: camera.CameraSettings,
        cal: Optional[kalkam.IntrinsicCalibration] = None,
        config: Optional[StarImageGeneratorConfig] = None,
    ):
        super().__init__(camera_settings, cal=cal, config=config)
        self._vector = self._rng.normal(size=(2, 3)) * 0.3

    @override
    def _capture(self) -> tuple[npt.NDArray[np.uint8], npt.NDArray[np.float64]]:
        self._vector *= 0.9
        self._vector += 0.1 * self._rng.normal(size=(2, 3)) * 0.3
        vector = self._vector + [[0, 0, 5], [0, 10, 0]]
        vector /= np.linalg.norm(vector, axis=-1, keepdims=True)
        image, star_xy, _ = self._sig(vector[0], vector[1], grid=self.grid)
        return image, star_xy


class AxisAlignCalibrationTestCam(ArtificialStarCam):
    """Star camera rotating around a random axis."""

    def __init__(
        self,
        camera_settings: camera.CameraSettings,
        cal: Optional[kalkam.IntrinsicCalibration] = None,
        config: Optional[StarImageGeneratorConfig] = None,
    ):
        super().__init__(camera_settings, cal=cal, config=config)
        vectors = self._rng.normal(size=(2, 3))
        vectors /= np.linalg.norm(vectors, axis=-1, keepdims=True)

        self.axis_vector = vectors[0]
        up_vector = vectors[1]

        rot_matrix = kalkam.look_at_extrinsic(self.axis_vector, [0, 0, 0], up_vector)[:3, :3]
        self.axis_attitude = scipy.spatial.transform.Rotation.from_matrix(rot_matrix)

        self.camera_rot = scipy.spatial.transform.Rotation.from_euler(
            "zxy", [2, 8, 12], degrees=True
        )

        self.axis_angle = 0.0

    @override
    def _capture(self) -> tuple[npt.NDArray[np.uint8], npt.NDArray[np.float64]]:
        self.axis_angle = (self.axis_angle + 0.1) % (2 * np.pi)
        self.axis_angle = (self.capture_time * const.EARTH_ANGULAR_VELOCITY) % (2 * np.pi)
        rotvec = np.array([0, 0, 1.0]) * self.axis_angle
        around_axis_rot = scipy.spatial.transform.Rotation.from_rotvec(rotvec, degrees=False)

        quat = (self.axis_attitude * around_axis_rot * self.camera_rot).as_quat(canonical=False)

        image, star_xy, _ = self._sig.image_from_quaternion(quat, grid=self.grid)
        return image, star_xy


class StarCameraCalibrationTestCam(ArtificialStarCam):
    """Star camera steadily facing a random direction."""

    theta: float
    """Elevation angle 0 is equatorial, pi/2 is to north pole, -pi/2 is to south pole."""
    epsilon: float
    """Roll component about the camera axis."""
    phi: float
    """Star angle, rotation about the north south axis."""

    def __init__(
        self,
        camera_settings: camera.CameraSettings,
        cal: Optional[kalkam.IntrinsicCalibration] = None,
        config: Optional[StarImageGeneratorConfig] = None,
    ):
        super().__init__(camera_settings, cal=cal, config=config)

        self.theta = self._rng.uniform(-np.pi / 2, np.pi / 2)
        self.epsilon = self._rng.uniform(-np.pi / 2, np.pi / 2)
        self.phi = 0

    def _get_extrinsic(self, t: float) -> np.ndarray:
        """Get the current extrinsic matrix of the camera.

        Changes with time.
        """
        self.phi = (t * const.EARTH_ANGULAR_VELOCITY) % (2 * np.pi)

        r = scipy.spatial.transform.Rotation.from_euler(
            "zxz", (self.epsilon, np.pi / 2 - self.theta, self.phi), degrees=False
        )

        rot_matrix = r.as_matrix()
        extrinsic = np.concatenate((rot_matrix.T, np.zeros((3, 1))), axis=1)
        return extrinsic

    @override
    def _capture(self) -> tuple[npt.NDArray[np.uint8], npt.NDArray[np.float64]]:
        extrinsic = self._get_extrinsic(self.capture_time)
        image, star_xy, _ = self._sig.image_from_extrinsic(extrinsic, grid=self.grid)
        return image, star_xy

    def gui(self) -> None:
        """Play with this camera in a GUI."""
        ct = CameraTester(self)
        ct.add_arg("t", 0, 60 * 60 * 24, init=0, dtype=float)
        ct.add_arg("theta", -np.pi / 2, np.pi / 2, dtype=float)
        ct.add_arg("epsilon", -np.pi, np.pi, dtype=float)
        ct.run()


if __name__ == "__main__":
    settings = camera.CameraSettings()
    cam = StarCameraCalibrationTestCam(settings)
    cam.theta = np.pi / 2
    cam.t = 0
    cam.grid = True

    cam.gui()
