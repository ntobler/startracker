"""Demo to show-case the star-based camera calibration.

Requires `startracker` to be installed with developer dependencies.
"""

import contextlib
import pathlib

import cv2
import matplotlib.pyplot as plt
import numpy as np
import scipy.spatial
from matplotlib.backends.backend_agg import FigureCanvasAgg

from startracker import (
    attitude_estimation,
    camera,
    const,
    initial_starcal,
    kalkam,
    testing_utils,
)
from startracker.demo import demo_utils


class Animation:
    """Animation of the internal works of StarCalibrator."""

    FONT_SIZE = 20
    MAX_N = 200

    def __init__(self) -> None:
        """Setup animation figure."""
        # Set up the figure to match 1920x1080 pixels exactly
        dpi = 100  # dots per inch
        width_in = 1920 / dpi
        height_in = 1080 / dpi
        self._fig, self._ax = plt.subplots(figsize=(width_in, height_in), dpi=dpi)
        self._canvas = FigureCanvasAgg(self._fig)
        self._ax.set_position((0, 0, 1, 1))  # fill the whole figure
        self._ax.axis("off")  # hide axes

        # Add axis elements
        self._image = self._ax.imshow(
            np.zeros((540, 960), dtype=np.uint8), cmap="gray", vmin=0, vmax=60
        )
        self._lat_lines = self._ax.plot(
            np.full((2, 36), np.nan),
            np.full((2, 36), np.nan),
            color="red",
            lw=2,
            alpha=0.6,
        )
        self._lon_lines = self._ax.plot(
            np.full((2, 72), np.nan),
            np.full((2, 72), np.nan),
            color="red",
            lw=2,
            alpha=0.6,
        )
        self._arrows = self._ax.quiver(
            np.full(self.MAX_N, 10),  # x positions
            np.full(self.MAX_N, 10),  # y positions
            np.full(self.MAX_N, 10),  # dx
            np.full(self.MAX_N, 10),  # dy
            angles="xy",
            scale_units="xy",
            scale=1,
            color="yellow",
            width=0.002,
            alpha=0.5,
            zorder=999,
        )

        self._detected_stars = self._ax.plot(
            np.full((2, 1), np.nan),
            np.full((2, 1), np.nan),
            ".",
            color="purple",
            alpha=0.3,
            markersize=12,
        )
        self._matched_stars = self._ax.plot(
            np.full((2, 1), np.nan),
            np.full((2, 1), np.nan),
            ".",
            color="green",
            alpha=0.3,
            markersize=12,
        )
        self._gradient_tracked_lines = self._ax.plot(
            np.full((2, self.MAX_N), np.nan),
            np.full((2, self.MAX_N), np.nan),
            color="white",
            lw=3,
            alpha=0.5,
        )

        margin = self.FONT_SIZE
        self._text = plt.text(
            margin, margin, "", fontsize=self.FONT_SIZE, va="top", ha="left", color="red"
        )

        legend_dict = {
            "Star movement tracking curves": self._gradient_tracked_lines[0],
            "Star movement vectors": self._arrows,
            "Celestial coordinate system": self._lat_lines[0],
            "Detected stars": self._detected_stars[0],
            "Matched stars": self._matched_stars[0],
        }
        legend = self._ax.legend(
            legend_dict.values(),
            legend_dict.keys(),
            loc="upper right",
            frameon=True,
            fontsize=self.FONT_SIZE,
        )
        legend.get_frame().set_facecolor("black")
        legend.get_frame().set_edgecolor("black")
        for text in legend.get_texts():
            text.set_color("white")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        plt.close(self._fig)
        with contextlib.suppress(cv2.error):
            cv2.destroyWindow("window")

    def update(
        self, image: np.ndarray, sc: initial_starcal.StarCalibrator, *, gui: bool = False
    ) -> np.ndarray:
        """Plot current state of the StarCalibrator object."""
        self._image.set_data(image)

        rot_correction = scipy.spatial.transform.Rotation.from_rotvec(
            [0, 0, -sc.times[-1] * const.EARTH_ANGULAR_VELOCITY], degrees=False
        ).as_matrix()

        intrinsic = sc.intrinsics[-1]
        extrinsic = sc.extrinsics[-1].copy()
        extrinsic[:3, :3] = extrinsic[:3, :3] @ rot_correction
        dist_coeffs = sc.dist_coeffs[-1]
        cal = kalkam.IntrinsicCalibration(intrinsic, dist_coeffs, sc.image_size)
        pp = kalkam.PointProjector(cal, extrinsic)

        latitudes = np.radians(np.arange(-90, 90.1, 5)), np.radians(np.arange(-90, 90.1, 1))
        longitudes = np.radians(np.arange(0, 360.1, 1)), np.radians(np.arange(0, 360.1, 5))

        cos_phi = cal.cos_phi(angle_margin_factor=1.2)
        for lats, lons, flip in zip(latitudes, longitudes, [False, True]):
            lat, lon = np.meshgrid(lats, lons)

            cos_lat = np.cos(lat)
            x = np.cos(lon) * cos_lat
            y = np.sin(lon) * cos_lat
            z = np.sin(lat)
            xyz_points = np.stack((x, y, z), axis=-1)

            # Get in-frame point
            target_vector = extrinsic[2, :3]
            mask = np.inner(xyz_points, target_vector) < cos_phi
            xy_points = pp.obj2pix(xyz_points, axis=-1)

            xy_points[mask] = np.nan

            if not flip:
                for i, line in enumerate(self._lat_lines):
                    line.set_xdata(xy_points[:, i, 0])
                    line.set_ydata(xy_points[:, i, 1])
            else:
                for i, line in enumerate(self._lon_lines):
                    line.set_xdata(xy_points[i, :, 0])
                    line.set_ydata(xy_points[i, :, 1])

        n_quivers = self._arrows.N
        offsets = np.full((n_quivers, 2), np.nan)
        u = np.full(n_quivers, np.nan)
        v = np.full(n_quivers, np.nan)
        stars_xy, stars_dxy, mses = sc.movement_registerer.get_results()
        if len(stars_xy):
            n = min(len(stars_xy), n_quivers)
            if n > 0:
                offsets[:n] = stars_xy[:n]
                u[:n] = stars_dxy[:n, 0] * 300
            v[:n] = stars_dxy[:n, 1] * 300
        self._arrows.set_offsets(offsets)
        self._arrows.set_UVC(u, v)

        candidates = sc.movement_registerer.candidates
        for i, line in enumerate(self._gradient_tracked_lines):
            if not sc.starbased_calibration_acquired and i < len(candidates):
                candidate = candidates[i]
                pos = np.array(candidate.positions)
                line.set_xdata(pos[:, 0])
                line.set_ydata(pos[:, 1])
            else:
                line.set_xdata([])
                line.set_ydata([])

        if sc.observed_stars_xy:
            detected_stars = np.concatenate(sc.observed_stars_xy, axis=0)
            self._detected_stars[0].set_xdata(detected_stars[:, 0])
            self._detected_stars[0].set_ydata(detected_stars[:, 1])

        if sc.image_xy_list:
            matched_stars = np.concatenate(sc.image_xy_list, axis=0)
            self._matched_stars[0].set_xdata(matched_stars[:, 0])
            self._matched_stars[0].set_ydata(matched_stars[:, 1])

        texts = [
            f"Star-gradient-based calibration acquired: {sc.first_estimate_acquired}",
            f"Star-based calibration acquired: {sc.starbased_calibration_acquired}",
            "Intrinsic camera matrix:",
            f"    [{intrinsic[0,0]:.1f}, {intrinsic[0,1]:.1f}, {intrinsic[0,2]:.1f}]",
            f"    [{intrinsic[1,0]:.1f}, {intrinsic[1,1]:.1f}, {intrinsic[1,2]:.1f}]",
            f"    [{intrinsic[2,0]:.1f}, {intrinsic[2,1]:.1f}, {intrinsic[2,2]:.1f}]",
            "Distortion coefficients:",
            f"    [{dist_coeffs[0]:.3f}, {dist_coeffs[1]:.3f}, {dist_coeffs[2]:.3f}, "
            f"{dist_coeffs[3]:.3f}, {dist_coeffs[4]:.3f}]",
        ]
        self._text.set_text("\n".join(texts))

        self._canvas.draw()
        buf = self._canvas.buffer_rgba()
        w, h = self._canvas.get_width_height()
        img_rgba = np.frombuffer(buf, dtype=np.uint8).reshape((h, w, 4))

        if gui:
            cv2.imshow("window", cv2.cvtColor(img_rgba, cv2.COLOR_RGBA2BGR))
            cv2.waitKey(10)

        return img_rgba


def demo():
    """Example script estimating camera calibration from artificial star data."""
    settings = camera.CameraSettings()
    config = testing_utils.StarImageGeneratorConfig(exposure=100, catalog_max_magnitude=7)
    cam = testing_utils.StarCameraCalibrationTestCam(settings, config=config)
    cam.theta = 0.5
    # cam.theta = -1.2
    # cam.theta = -1.4
    # cam.theta = 0.0
    # cam.cal.dist_coeffs[1:] *= 0.1

    ae_config = attitude_estimation.AttitudeEstimatorConfig(
        star_match_pixel_tol=5, n_match=10, max_candidates=100
    )
    star_cal_config = initial_starcal.StarCalibratorConfig(
        ae_config, interval=1, movement_register_mse_threshold=2
    )
    sc = initial_starcal.StarCalibrator(star_cal_config, (960, 540))

    vc = demo_utils.VideoCreator()

    with Animation() as anim:
        for t in list(range(0, 60 * 5, 10)):
            cam.t = t
            image = cam.capture()
            sc.put_image(image, t)
            frame = anim.update(image, sc, gui=False)
            vc.push(frame)

    vc.save_webp(pathlib.Path("video.webp"))
    vc.save_mp4(pathlib.Path("video.mp4"))

    sc.plot(gt_intrinsic=cam.cal.intrinsic, gt_dist_coeffs=cam.cal.dist_coeffs)


if __name__ == "__main__":
    demo()
