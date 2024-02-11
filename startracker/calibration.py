"""Camera Calibration."""

import pathlib
import json
import collections

import cv2
import numpy as np

from . import kalkam

from typing import Union, Sequence, Dict


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

    def save_json(self, filename: Union[str, pathlib.Path]):
        """
        Save calibration to JSON.

        Args:
            filename: JSON file name
        """
        with open(filename, "w") as f:
            json.dump(self.cal_dict, f)

    @classmethod
    def make_from_images(cls, files: Sequence[Union[str, pathlib.Path]]):
        pattern = kalkam.ChArUcoPattern(19, 11, int(283 / 6))
        cal = kalkam.Calibration.make_from_image_files(files, pattern, plot=True)

        # dist coeffs are always present if made from calibration
        assert cal.dist_coeffs is not None

        cal_dict = {
            "camera_matrix": cal.intrinsic.tolist(),
            "dist_coefs": cal.dist_coeffs.tolist(),
            "resolution": list(cv2.imread(str(files[0])).shape[:2][::-1]),
            "camera_model": "Brown",
            "k1": cal.dist_coeffs[0, 0].item(),
            "k2": cal.dist_coeffs[0, 1].item(),
            "k3": cal.dist_coeffs[0, 4].item(),
            "p1": cal.dist_coeffs[0, 2].item(),
            "p2": cal.dist_coeffs[0, 3].item(),
            "fx": cal.intrinsic[0, 0].item(),
            "fy": cal.intrinsic[1, 1].item(),
            "up": cal.intrinsic[0, 2].item(),
            "vp": cal.intrinsic[1, 2].item(),
            "skew": cal.intrinsic[0, 1].item(),
            "RMS_reproj_err_pix": cal.rms_error,
        }
        return cls(cal_dict)
