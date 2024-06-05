"""Various helper function and classes for testing, especially for star and camera simulation."""

import dataclasses
import pathlib
import time
from typing import Final, List, Optional, Tuple

import cots_star_tracker
import cv2
import numpy as np
import scipy.spatial.transform

from startracker import (
    attitude_estimation,
    calibration,
    camera,
    image_utils,
    kalkam,
    persistent,
    transform,
)

UINT8_MAX = 255


class TestingMaterial:
    testing_dir: pathlib.Path
    cam_file: pathlib.Path
    stardata_dir: pathlib.Path

    def __init__(self, *, use_existing: bool = True):
        user_data_dir = persistent.Persistent.get_instance().user_data_dir

        self.testing_dir = user_data_dir / "testing"
        self.testing_dir.mkdir(exist_ok=True)

        cal = calibration.make_dummy()

        self.cam_file = self.testing_dir / "calibration.json"
        if (not self.cam_file.exists()) or (not use_existing):
            cal.to_json(self.cam_file)

        self.stardata_dir = self.testing_dir / "stardata"
        if (not self.stardata_dir.exists()) or (not use_existing):
            self.stardata_dir.mkdir(exist_ok=True)
            attitude_estimation.create_catalog(
                cal, self.stardata_dir, magnitude_threshold=5.5, verbose=True
            )

    def patch_persistent(self):
        persistent.Persistent.get_instance().cam_file = self.cam_file
        persistent.Persistent.get_instance().star_data_dir = self.stardata_dir


def get_catalog_stars() -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Get star location and magnitude."""
    u, mag, _ = cots_star_tracker.read_star_catalog(cots_star_tracker.get_star_cat_file(), 8)
    u /= np.linalg.norm(u, axis=-0, keepdims=True)
    az, el = transform.nwu2azel(u, axis=0)
    return az, el, mag


class StarImageGenerator:
    def __init__(
        self,
        cal: kalkam.IntrinsicCalibration,
        exposure: float = 15.0,
        blur: float = 0.7,
        noise_sigma: float = 1.0,
        black_level: float = 4.5,
    ):
        self._cal = cal
        self.width, self.height = cal.image_size
        self.intrinsic = cal.intrinsic

        self.exposure = exposure
        self.blur = blur
        self.noise_sigma = noise_sigma
        self.black_level = black_level

        self._cos_phi = self._cal.cos_phi(angle_margin_factor=1.2)

        if self._cal.dist_coeffs is not None:
            self.distorter = kalkam.PointUndistorter(self._cal)
        else:
            self.distorter = None

        az, el, self.stars_mags = get_catalog_stars()
        self.stars_nwu = transform.azel2nwu(np.stack((az, el), axis=-1))

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
        target_vector: np.ndarray,
        up_vector: np.ndarray,
        *,
        grid: bool = False,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Create image of stars.

        Args:
            target_vector: shape=[3] Equatorical target vector, the camera points at in astronomical
                coordinate system e.g. north=[0,0,1]
            up_vector: shape=[3] Up-facing vector of the camera in astronomical coordinate system
                e.g. north=[0,0,1]
            grid: Display longitude and latitude overlay

        Returns:
            Tuple[np.ndarray, np.ndarray, np.ndarray]:
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
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        rot = scipy.spatial.transform.Rotation.from_quat(quat)
        extrinsic = np.concatenate((rot.as_matrix().T, np.zeros((3, 1))), axis=-1)
        return self.image_from_extrinsic(extrinsic, grid=grid)

    def stars_from_extrinsic(self, extrinsic: np.ndarray):
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

    def image_from_extrinsic(self, extrinsic: np.ndarray, *, grid: bool = False):
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
            canvas = cv2.GaussianBlur(canvas, (25, 25), self.blur)
        if self.noise_sigma is not None:
            canvas += self._rng.normal(
                size=canvas.shape, scale=self.noise_sigma, loc=self.black_level
            )

        # Clip image and convert image to uint8
        canvas = np.clip(canvas, 0, UINT8_MAX)
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
        dtype: float

    sliders: List[Slider]

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
        dtype=int,
    ):
        """Add slider for a numerial attribute of the camera."""
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
            ax = plt.axes([0.1, pos, 0.65, 0.03], facecolor="lightgoldenrodyellow")
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

    cal: Final[kalkam.IntrinsicCalibration]
    """Calibration used to create mock images"""
    t: Optional[float] = None
    """Capture time in seconds."""
    grid: bool = False
    """Display longitude and latitude grid overlay."""

    def __init__(self, camera_settings: camera.CameraSettings):
        super().__init__(camera_settings)
        cam_file = TestingMaterial(use_existing=True).cam_file
        self._rng = np.random.default_rng(42)
        self.cal = kalkam.IntrinsicCalibration.from_json(cam_file)
        self._sig = StarImageGenerator(self.cal)

    def capture_raw(self):
        return self.capture()

    def record_darkframe(self):
        pass

    def gui(self):
        """Play with this camera in a GUI."""
        ct = CameraTester(self)
        ct.add_arg("t", 0, 60 * 60 * 24, init=0, dtype=float)
        ct.run()


class RandomStarCam(ArtificialStarCam):
    """Star camera pointing in a random direction and wiggeling."""

    def __init__(self, camera_settings: camera.CameraSettings):
        super().__init__(camera_settings)
        self._vector = self._rng.normal(size=(2, 3)) * 0.3

    def capture(self) -> np.ndarray:
        self._vector *= 0.9
        self._vector += 0.1 * self._rng.normal(size=(2, 3)) * 0.3
        vector = self._vector + [[0, 0, 5], [0, 10, 0]]
        vector /= np.linalg.norm(vector, axis=-1, keepdims=True)
        image, _, _ = self._sig(vector[0], vector[1], grid=self.grid)
        return image


class AxisAlignCalibrationTestCam(ArtificialStarCam):
    """Star camera rotating around a random axis."""

    def __init__(self, camera_settings: camera.CameraSettings):
        super().__init__(camera_settings)
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

    def capture(self) -> np.ndarray:
        t = time.monotonic() if self.t is None else self.t

        self.axis_angle = (self.axis_angle + 0.1) % (2 * np.pi)
        self.axis_angle = (t * ((2 * np.pi) / (24 * 60 * 60))) % (2 * np.pi)
        rotvec = np.array([0, 0, 1.0]) * self.axis_angle
        around_axis_rot = scipy.spatial.transform.Rotation.from_rotvec(rotvec, degrees=False)

        quat = (self.axis_attitude * around_axis_rot * self.camera_rot).as_quat()

        image, _, _ = self._sig.image_from_quaternion(quat, grid=self.grid)
        return image


class StarCameraCalibrationTestCam(ArtificialStarCam):
    """Star camera steadily facing a random direction."""

    theta: float
    """Elevation angle 0 is equatorial, pi/2 is to north pole, -pi/2 is to south pole."""
    epsilon: float
    """Roll component about the camera axis."""
    phi: float
    """Star angle, rotation about the north sourth axis."""

    def __init__(self, camera_settings: camera.CameraSettings):
        super().__init__(camera_settings)

        self.theta = self._rng.uniform(-np.pi / 2, np.pi / 2)
        self.epsilon = self._rng.uniform(-np.pi / 2, np.pi / 2)
        self.phi = 0

    def get_extrinsic(self) -> np.ndarray:
        t = time.monotonic() if self.t is None else self.t

        self.phi = (t * ((2 * np.pi) / (24 * 60 * 60))) % (2 * np.pi)

        r = scipy.spatial.transform.Rotation.from_euler(
            "zxz", (self.epsilon, np.pi / 2 - self.theta, self.phi), degrees=False
        )

        rot_matrix = r.as_matrix()
        extrinsic = np.concatenate((rot_matrix.T, np.zeros((3, 1))), axis=1)
        return extrinsic

    def capture(self) -> np.ndarray:
        extrinsic = self.get_extrinsic()
        image, _, _ = self._sig.image_from_extrinsic(extrinsic, grid=self.grid)
        return image

    def gui(self):
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
