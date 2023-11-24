import glob
import pathlib
from datetime import datetime
import time
import pickle

import numpy as np
import cv2
import scipy.spatial.transform
import matplotlib.pyplot as plt
import cots_star_tracker

from . import kalkam
from . import attitude_estimation
from . import image_processing
from . import calibration
from . import persistent

from typing import Tuple


def get_catalog_stars() -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Get star location and magnitude."""
    u, mag, _ = cots_star_tracker.read_star_catalog(
        cots_star_tracker.get_star_cat_file(), 8
    )
    u /= np.linalg.norm(u, axis=-0, keepdims=True)
    az, el = nwu2azel(u, axis=0)
    return az, el, mag


def azel2nwu(az_el: np.ndarray, axis=-1, degrees: bool = False) -> np.ndarray:
    """Convert azimuth, elevation to a unit vector in the north west up frame."""
    if degrees:
        az_el = np.radians(az_el)
    az, el = np.moveaxis(az_el, axis, 0)
    z = np.sin(el)
    cos_el = np.cos(el)
    x = np.cos(az)
    x *= cos_el
    y = np.sin(-az)
    y *= cos_el
    nwu = np.stack((x, y, z), axis=axis)
    return nwu


def nwu2azel(nwu: np.ndarray, axis=-1, degrees: bool = False):
    """Convert north west up coordiantes to azimuth, elevation."""
    nwu = np.moveaxis(nwu, axis, 0)
    x, y, z = nwu
    az = np.arctan2(-y, x)
    el = np.arctan2(z, np.linalg.norm(nwu[:2], axis=0))
    az += (az < 0) * (np.pi * 2)
    az_el = np.stack((az, el), axis=axis)
    if degrees:
        az_el *= 180 / np.pi
    return az_el


def test_azel_nwu():
    rng = np.random.default_rng(42)

    nwu = rng.normal(size=(10, 10, 3))
    nwu /= np.linalg.norm(nwu, axis=-1, keepdims=True)

    res = azel2nwu(nwu2azel(nwu))
    res /= np.linalg.norm(res, axis=-1, keepdims=True)

    print(np.allclose(nwu[..., 0], res[..., 0]))
    print(np.allclose(nwu[..., 1], res[..., 1]))
    print(np.allclose(nwu[..., 2], res[..., 2]))
    assert np.allclose(nwu, res)


class StarImageGenerator:
    def __init__(self, intrinsic=None, imsize=None, dist_coeffs=None):
        if imsize is None:
            imsize = (1920, 1080)

        self.width, self.height = imsize

        self.exposure = 15
        self.blur = 0.7
        self.noise_sigma = 1
        self.black_level = 4.5

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
        self.stars_nwu = azel2nwu(np.stack((az, el), axis=-1))

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
        self, target_vector: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Create image of stars.

        Args:
            target_vector: shape=[3] Target vector, the camera points at in astronomical
                coordinate system e.g. north=[0,0,1]

        Returns:
            Tuple[np.ndarray, np.ndarray, np.ndarray]:
                - Grayscale uint8 star image
                - Painted star coordinates in pixels shape = [n, 2 (x, y)]
                - Painted star magnitudes
        """

        # Build extrinsic
        target_vector = np.array(target_vector, np.float32)
        target_vector /= np.linalg.norm(target_vector, axis=0)
        up_vector = np.array([0, 0.01, 1])
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


def get_test_context():
    testing_dir = persistent.Persistent.get_instance().testing_dir
    cam_file = testing_dir / "calibration.json"
    data_dir = testing_dir / "stardata"
    data_dir.mkdir(exist_ok=True)

    calibration.CameraCalibration.make_dummy().save_json(cam_file)
    cots_star_tracker.create_catalog(cam_file, data_dir, b_thresh=5.5, verbose=True)
    return data_dir, cam_file


def test_attutude_estimation():
    data_dir, cam_file = get_test_context()

    rng = np.random.default_rng(42)

    ae = attitude_estimation.AttitudeEstimator(cam_file, data_dir)
    intrinsic, (width, height), dist_coeffs = cots_star_tracker.read_cam_json(cam_file)

    sig = StarImageGenerator(intrinsic, (width, height), dist_coeffs)

    vectors = rng.normal(size=(100, 3))
    vectors /= np.linalg.norm(vectors, axis=-1, keepdims=True)

    for vector in vectors:
        image, gt_xy, mag = sig(vector)

        quat, n_matches, image_xyz, cat_xyz = ae(image)

        image_xy = (sig.intrinsic @ image_xyz.T).T
        image_xy = image_xy[..., :2] / image_xy[..., 2:]
        if sig.distorter is not None:
            image_xy = sig.distorter.distort(image_xy)

        cat_xy = (sig.intrinsic @ cat_xyz.T).T
        cat_xy = cat_xy[..., :2] / cat_xy[..., 2:]
        if sig.distorter is not None:
            cat_xy = sig.distorter.distort(cat_xy)

        rot = scipy.spatial.transform.Rotation.from_quat(quat)
        res = rot.apply([0, 0, 1])

        delta_angle = np.degrees(np.arccos(np.dot(vector, res)))

        assert delta_angle < 0.1

        if False:
            import matplotlib.pyplot as plt

            fig, ax = plt.subplots(1, constrained_layout=False)
            ax.imshow(image)
            ax.plot(gt_xy[..., 0], gt_xy[..., 1], "x")
            ax.plot(image_xy[..., 0], image_xy[..., 1], "x")
            ax.plot(cat_xy[..., 0], cat_xy[..., 1], "x")
            for (x, y), m in zip(gt_xy, mag):
                ax.text(x, y, f" {m:.1f}")
            fig.suptitle(
                f"quat: {quat}, n_matches: {n_matches}, delta_angle: {delta_angle:.3f}"
            )
            plt.show()

