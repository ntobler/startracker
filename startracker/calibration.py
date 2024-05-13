"""Camera Calibration."""

import pathlib
import json
import collections

import numpy as np

from . import kalkam

from typing import Union, Dict, Optional


IntrinsicParams = collections.namedtuple(
    "IntrinsicParams", ["fx", "fy", "tx", "ty", "width", "height"]
)


class CameraCalibration:
    def __init__(self, cal_dict: Dict):
        """
        Camera Calibration that can be saved and restored from the filesystem.

        Args:
            cal_dict: Dictionary holding all camera calibration data.
        """
        self.cal_dict = cal_dict

    @classmethod
    def make_dummy(cls) -> "CameraCalibration":
        """
        Return dummy calibration that may be used for testing.

        Returns:
            CameraCalibration: Calibration instance with dummy values.
        """
        cal_dict = {
            "camera_matrix": [
                [2126.9433827817543, 0.0, 471.5660270698475],
                [0.0, 2125.4878604007063, 310.73929485280405],
                [0.0, 0.0, 1.0],
            ],
            "dist_coefs": [
                [
                    -0.4412080745099301,
                    -0.159542022464492,
                    0.007670124986859448,
                    -0.002132920872628578,
                    -1.6478824652916775,
                ]
            ],
            "resolution": [960, 540],
            "camera_model": "Brown",
            "k1": -0.4412080745099301,
            "k2": -0.159542022464492,
            "k3": -1.6478824652916775,
            "p1": 0.007670124986859448,
            "p2": -0.002132920872628578,
            "fx": 2126.9433827817543,
            "fy": 2125.4878604007063,
            "up": 471.5660270698475,
            "vp": 310.73929485280405,
            "skew": 0.0,
            "RMS_reproj_err_pix": 0.5855461668735685,
        }
        return cls(cal_dict)

    def intrinsic_params(self) -> IntrinsicParams:
        """
        Get intrinsic camera parameters.

        Returns:
            IntrinsicParams: Named tuple containing camera parameters
        """
        intrinsic = self.cal_dict["camera_matrix"]
        fx = intrinsic[0][0]
        fy = intrinsic[1][1]
        tx = intrinsic[0][2]
        ty = intrinsic[1][2]
        width, height = self.cal_dict["resolution"]
        return IntrinsicParams(fx, fy, tx, ty, width, height)

    def get_frame_corners(self) -> np.ndarray:
        """
        Get a numpy array representing the corners of the camera in cartesian space.

        Returns:
            np.ndarray: Corners xyz shape=[5, 3]
        """
        p = self.intrinsic_params()
        x0 = (-p.tx - 0.5) / p.fx
        x1 = (p.width - p.tx - 0.5) / p.fx
        y0 = (-p.ty - 0.5) / p.fy
        y1 = (p.height - p.ty - 0.5) / p.fy

        corners = np.array(
            ((x0, x1, x1, x0, x0), (y0, y0, y1, y1, y0), (1, 1, 1, 1, 1)),
            dtype=np.float32,
        ).T
        return corners

    def get_distorted_camera_frame(self, segments_per_side: int = 10) -> np.ndarray:
        """
        Get a numpy array representing a polygon of the distorted camera frame in cartesian space.

        Returns:
            np.ndarray: Points xyz shape=[segments_per_side * 4, 3]
        """
        xf = np.arange(segments_per_side * 4) / segments_per_side

        p = self.intrinsic_params()
        x0 = -0.5
        x1 = p.width - 0.5
        y0 = -0.5
        y1 = p.height - 0.5

        corners = np.array(
            ((x0, x1, x1, x0, x0), (y0, y0, y1, y1, y0), (1, 1, 1, 1, 1)),
            dtype=np.float32,
        )
        points = np.array([np.interp(xf, np.arange(len(d)), d) for d in corners]).T

        c = kalkam.PointUndistorter(
            kalkam.IntrinsicCalibration(
                np.array(self.cal_dict["camera_matrix"]),
                np.array(self.cal_dict["dist_coefs"]),
                (p.width, p.height),
            )  # TODO Ewwwwww fix this
        ).undisort(points[:, :2])

        points[:, :2] = (c - [p.tx, p.ty]) / [p.fx, p.fy]

        return points

    def save_json(self, filename: Union[str, pathlib.Path]):
        """
        Save calibration to JSON.

        Args:
            filename: JSON file name
        """
        with open(filename, "w") as f:
            json.dump(self.cal_dict, f)

    @classmethod
    def from_json(cls, filename: Union[str, pathlib.Path]):
        """
        Create calibration from JSON.

        Args:
            filename: JSON file name
        """
        with open(filename, "r") as f:
            cal_dict = json.load(f)
        return CameraCalibration(cal_dict)

    @classmethod
    def make(
        cls,
        intrinsic: np.ndarray,
        width: int,
        height: int,
        dist_coeffs: Optional[np.ndarray] = None,
        error: float = np.nan,
    ) -> "CameraCalibration":
        """
        Return calibration from given parameters.

        Args:
            intrinsic: 3x3 intrinsic camera matrix
            width: Width of the camera frame in pixels
            height: Height of the camera frame in pixels
            dist_coeffs: Distortion coefficients as defined by OpenCV
            error: reprojection error

        Returns:
            CameraCalibration: Calibration instance with dummy values.
        """

        if dist_coeffs is None:
            dist_coeffs = np.zeros((1, 5), dtype=np.float32)

        cal_dict = {
            "camera_matrix": intrinsic.tolist(),
            "dist_coefs": dist_coeffs.tolist(),
            "resolution": [width, height],
            "camera_model": "Brown",
            "k1": dist_coeffs[0, 0].item(),
            "k2": dist_coeffs[0, 1].item(),
            "k3": dist_coeffs[0, 4].item(),
            "p1": dist_coeffs[0, 2].item(),
            "p2": dist_coeffs[0, 3].item(),
            "fx": intrinsic[0, 0].item(),
            "fy": intrinsic[1, 1].item(),
            "up": intrinsic[0, 2].item(),
            "vp": intrinsic[1, 2].item(),
            "skew": intrinsic[0, 1].item(),
            "RMS_reproj_err_pix": error,
        }
        return cls(cal_dict)
