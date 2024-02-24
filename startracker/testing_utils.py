"""Various helper function and classes for testing, especially for star and camera simulation"""

import pathlib

import cots_star_tracker
import cv2
import numpy as np
import scipy.spatial.transform

from startracker import calibration
from startracker import kalkam
from startracker import persistent
from startracker import transform
from startracker import camera

from typing import Tuple, Literal, Optional


class TestingMaterial:

    testing_dir: pathlib.Path
    cam_file: pathlib.Path
    stardata_dir: pathlib.Path

    def __init__(self, use_existing: bool = True):

        user_data_dir = persistent.Persistent.get_instance().user_data_dir

        self.testing_dir = user_data_dir / "testing"
        self.testing_dir.mkdir(exist_ok=True)

        self.cam_file = self.testing_dir / "calibration.json"
        if (not self.cam_file.exists()) or (not use_existing):
            calibration.CameraCalibration.make_dummy().save_json(self.cam_file)

        self.stardata_dir = self.testing_dir / "stardata"
        if (not self.stardata_dir.exists()) or (not use_existing):
            self.stardata_dir.mkdir(exist_ok=True)
            cots_star_tracker.create_catalog(
                self.cam_file, self.stardata_dir, b_thresh=5.5, verbose=True
            )

    def patch_persistent(self):
        persistent.Persistent.get_instance().cam_file = self.cam_file
        persistent.Persistent.get_instance().star_data_dir = self.stardata_dir


def get_catalog_stars() -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Get star location and magnitude."""
    u, mag, _ = cots_star_tracker.read_star_catalog(
        cots_star_tracker.get_star_cat_file(), 8
    )
    u /= np.linalg.norm(u, axis=-0, keepdims=True)
    az, el = transform.nwu2azel(u, axis=0)
    return az, el, mag


class StarImageGenerator:
    def __init__(
        self,
        intrinsic: Optional[np.ndarray] = None,
        imsize: Optional[Tuple[int, int]] = None,
        dist_coeffs: Optional[np.ndarray] = None,
        exposure: float = 15.0,
        blur: float = 0.7,
        noise_sigma: float = 1.0,
        black_level: float = 4.5,
    ):
        if imsize is None:
            imsize = (1920, 1080)

        self.width, self.height = imsize
        self.exposure = exposure
        self.blur = blur
        self.noise_sigma = noise_sigma
        self.black_level = black_level

        if intrinsic is None:
            lens_focal_distance = 12
            sensor_diagonal = 6.4
            intrinsic = kalkam.intrinsic_from_camera_param(
                lens_focal_distance, sensor_diagonal, self.width, self.height
            )
        self.intrinsic = intrinsic

        if dist_coeffs is not None:
            self.distorter = kalkam.PointUndistorter(
                kalkam.Calibration(intrinsic, dist_coeffs, 0, 0)
            )
        else:
            self.distorter = None

        az, el, self.stars_mags = get_catalog_stars()
        self.stars_nwu = transform.azel2nwu(np.stack((az, el), axis=-1))

        ANGLE_MARGIN_FACTOR = 1.2
        self._cos_phi = np.cos(
            ANGLE_MARGIN_FACTOR
            * np.arctan(np.linalg.norm(imsize) / intrinsic[0, 0] / 2)
        )

        self._rng = np.random.default_rng(42)

        self._offset = np.array(
            [
                [[0, 0], [0, 1]],
                [[1, 0], [1, 1]],
            ],
            dtype=np.uint32,
        )

    def __call__(
        self, target_vector: np.ndarray, up_vector: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Create image of stars.

        Args:
            target_vector: shape=[3] Equatorical target vector, the camera points at in astronomical
                coordinate system e.g. north=[0,0,1]
            up_vector: shape=[3] Up-facing vector of the camera in astronomical coordinate system
                e.g. north=[0,0,1]

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

        return self.image_from_extrinsic(extrinsic)

    def image_from_quaternion(
        self, quat: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        rot = scipy.spatial.transform.Rotation.from_quat(quat)
        extrinsic = np.concatenate((rot.as_matrix().T, np.zeros((3, 1))), axis=-1)
        return self.image_from_extrinsic(extrinsic)

    def image_from_extrinsic(self, extrinsic):
        UINT8_MAX = 255

        width, height = self.width, self.height

        # Take z image vector of the inverted extrinsic
        target_vector = extrinsic[2, :3]

        # Select all stars that are roughly in frame
        in_frame = np.inner(self.stars_nwu, target_vector) > self._cos_phi
        stars_nwu = self.stars_nwu[in_frame]
        stars_mags = self.stars_mags[in_frame]

        # Convert star magnitude (visible flux) to a pixel value (may be way above 255)
        star_brightness = (100 ** (1 / 5)) ** (-stars_mags)
        star_brightness = star_brightness * (self.exposure * UINT8_MAX)

        # Convert stars to pixels
        pp = kalkam.PointProjector(intrinsic=self.intrinsic, extrinsic=extrinsic)
        stars_xy = pp.obj2pix(stars_nwu, axis=-1)
        if self.distorter is not None:
            stars_xy = self.distorter.distort(pp.obj2pix(stars_nwu, axis=-1))

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

        # Distribute the brightness to each pixel of the quad according to the distance
        pixel_mags = np.broadcast_to(
            star_brightness[..., None, None], (len(stars_xy), 2, 2)
        ).copy()
        pixel_mags[..., ::2, :] *= 1 - pixel_remainder[..., None, None, 0]
        pixel_mags[..., 1::2, :] *= pixel_remainder[..., None, None, 0]
        pixel_mags[..., :, ::2] *= 1 - pixel_remainder[..., None, None, 1]
        pixel_mags[..., :, 1::2] *= pixel_remainder[..., None, None, 1]
        pixel_mags = pixel_mags.ravel()

        # Mask pixels that are in frame, now accurately
        mask = np.logical_and(pixel_coords >= 0, pixel_coords < (width, height)).all(
            axis=-1
        )
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

        return canvas, stars_xy, stars_mags


class MockStarCam:
    """Camera to generate artificial images of the sky"""

    def __init__(self):
        cam_file = TestingMaterial(use_existing=True).cam_file
        self._rng = np.random.default_rng(42)
        intrinsic, (width, height), dist_coeffs = cots_star_tracker.read_cam_json(
            cam_file
        )
        self._sig = StarImageGenerator(intrinsic, (width, height), dist_coeffs)


class RandomStarCam(MockStarCam):
    """Star camera pointing in a random direction and wiggeling."""

    def __init__(self):
        super().__init__()
        self._vector = self._rng.normal(size=(2, 3)) * 0.3

    def __call__(self) -> np.ndarray:
        self._vector *= 0.9
        self._vector += 0.1 * self._rng.normal(size=(2, 3)) * 0.3
        vector = self._vector + [[0, 0, 5], [0, 10, 0]]
        vector /= np.linalg.norm(vector, axis=-1, keepdims=True)
        image, _, _ = self._sig(vector[0], vector[1])
        return image


class AxisAlignCalibrationTestCam(MockStarCam):
    """Star camera rotating around a random axis."""

    def __init__(self):
        super().__init__()
        vectors = self._rng.normal(size=(2, 3))
        vectors /= np.linalg.norm(vectors, axis=-1, keepdims=True)

        self.axis_vector = vectors[0]
        up_vector = vectors[1]

        rot_matrix = kalkam.look_at_extrinsic(self.axis_vector, [0, 0, 0], up_vector)[
            :3, :3
        ]
        self.axis_attitude = scipy.spatial.transform.Rotation.from_matrix(rot_matrix)
        self.align_error_deg = self.axis_attitude.magnitude() * (180 / np.pi)

        self.camera_rot = scipy.spatial.transform.Rotation.from_euler(
            "zxy", [2, 8, 12], degrees=True
        )

        self.axis_angle = 0.0

    def __call__(self) -> np.ndarray:

        self.axis_angle = (self.axis_angle + 0.1) % (2 * np.pi)
        rotvec = np.array([0, 0, 1.0]) * self.axis_angle
        around_axis_rot = scipy.spatial.transform.Rotation.from_rotvec(
            rotvec, degrees=False
        )

        quat = (self.axis_attitude * around_axis_rot * self.camera_rot).as_quat()

        image, _, _ = self._sig.image_from_quaternion(quat)
        return image


class DebugCamera(camera.Camera):
    """Camera to generate artificial images of the sky"""

    mode: Literal["stars", "axis_align_calibration"] = "stars"

    def __init__(
        self,
        camera_settings: camera.CameraSettings,
        mode: Optional[Literal["stars"]] = None,
    ):
        super().__init__(camera_settings)
        if mode is not None:
            self.mode = mode
        self._functions = {
            "stars": RandomStarCam(),
            "axis_align_calibration": AxisAlignCalibrationTestCam(),
        }

    def capture_raw(self):
        return self.capture()

    def capture(self) -> np.ndarray:
        frame = self._functions[self.mode]()
        return frame

    def record_darkframe(self):
        pass
