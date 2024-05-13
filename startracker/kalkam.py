"""Kalibrated Kamera module."""

import abc
import pathlib
import json
import time
import dataclasses

import numpy as np
import cairo
import cv2

from typing import Union, Tuple, Iterable, Optional, Sequence, List
from numpy.typing import ArrayLike


def _parse_version(version_str: str) -> int:
    version_float = 0
    for i, x in enumerate(version_str.split(".")):
        version_float += int(x) * 10 ** (-5 * i)
    return version_float


class CalibrationPattern(abc.ABC):
    """Abstract Calibration Pattern super class."""

    width: int
    """Number of squares in the horizontal x direction."""
    height: int
    """Number of squares in the vertical y direction."""
    square_size: float
    """Size of a square in millimeters."""
    overall_size: Tuple[float, float]
    """Dimensions (x, y) of the whole pattern in millimeters."""
    object_points: np.ndarray
    """3D object coordinates of all markers, shape=[n_markers, 3]."""

    def save(self, filename: Union[pathlib.Path, str]):
        """
        Save Pattern definition as JSON file.

        Args:
            filename: Json file path.
        """
        with open(filename, "w") as f:
            json.dump(self.to_dict(), f)

    @classmethod
    def load(cls, filename: Union[pathlib.Path, str]) -> "CalibrationPattern":
        """
        Load a pattern from a JSON file.

        Args:
            filename: Json file path.

        Returns:
            CalibrationPattern: loaded pattern instance.
        """
        with open(filename, "r") as f:
            d = json.load(f)
        classes = [cls]
        class_map = {c.__name__: c for c in classes}
        load_class = class_map.get(d["type"], None)
        if load_class is None:
            raise ValueError(f"Unknown Calibration pattern: {d['type']}")
        return load_class.from_dict(d)

    @abc.abstractmethod
    def to_dict(self) -> dict:
        """Get definition dictionary."""

    @classmethod
    @abc.abstractmethod
    def from_dict(cls, d: dict) -> "CalibrationPattern":
        """
        Create pattern from a dictionary definition.

        Args:
            d: Definition dictionary.

        Returns:
            CalibrationPattern: loaded pattern instance.
        """

    @abc.abstractmethod
    def find_in_image(
        self, image: np.ndarray, plot: bool = False
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Find the pattern in the given image.

        Args:
            image: Grayscale or color image
            plot: Show matplotlib plot of the detection. Defaults to False.

        Raises:
            ValueError: if pattern is not found

        Returns:
            Tuple[np.ndarray, np.ndarray]:
                - 2D pixel coordinates, shape=[n_detected_points, 2]
                - 3D object coordinates, shape=[n_detected_points, 3]
        """

    def export_svg(self, filename: Union[pathlib.Path, str]):
        """
        Export the pattern as a CSV graphic.

        Args:
            filename: SVG file name.
        """
        with cairo.SVGSurface(
            str(filename), self.overall_size[0], self.overall_size[1]
        ) as surface:
            surface.set_document_unit(cairo.SVG_UNIT_MM)
            ctx = cairo.Context(surface)

            ctx.set_source_rgb(0, 0, 0)
            ctx.set_line_width(0)

            self.draw_cairo(ctx)

    def export_pdf(
        self,
        filename: Union[pathlib.Path, str],
        page_width: float = 297.0,
        page_height: float = 210.0,
    ):
        """
        Export the pattern as a CSV graphic.

        The pattern is centered on the page.

        Args:
            filename: SVG file name.
            page_width: Page width in millimeters
            page_height: Page height in millimeters
        """
        f = 72.0 / 25.4  # cairo pdf surface uses points, where 72 points are an inch

        with cairo.PDFSurface(
            str(filename), page_width * f, page_height * f
        ) as surface:
            ctx = cairo.Context(surface)

            ctx.set_source_rgb(0, 0, 0)
            ctx.set_line_width(0)

            ctx.scale(f, f)

            ctx.translate(page_width / 2, page_height / 2)
            ctx.translate(-self.overall_size[0] / 2, -self.overall_size[1] / 2)

            self.draw_cairo(ctx)

    def export_png(
        self,
        filename: Union[pathlib.Path, str],
        image_width: int,
        image_height: int,
        dpi: float = 72.0,
    ):
        """
        Export the pattern as a PNG image.

        The pattern is centered in the image.

        Args:
            filename: PNG file name.
            image_width: Image width in pixels
            image_height: Image height in pixels
            dpi: Pixel per inch
        """
        pix_per_mm = dpi / 25.4

        width_pixel = self.width * self.square_size * pix_per_mm
        height_pixel = self.height * self.square_size * pix_per_mm

        if (width_pixel > image_width) or (height_pixel > image_height):
            raise ValueError("Pattern does not fit into screen")

        canvas = np.zeros((image_height, image_width), dtype=np.uint8)
        surface = cairo.ImageSurface.create_for_data(
            canvas.data,
            cairo.FORMAT_A8,
            image_width,
            image_height,
        )
        ctx = cairo.Context(surface)

        ctx.set_source_rgba(1, 1, 1, 1)
        ctx.set_line_width(0)

        ctx.translate(image_width / 2, image_height / 2)
        ctx.scale(pix_per_mm, pix_per_mm)
        ctx.translate(-self.overall_size[0] / 2, -self.overall_size[1] / 2)

        self.draw_cairo(ctx)

        cv2.imwrite(str(filename), 255 - canvas)

    @abc.abstractmethod
    def draw_cairo(self, ctx: cairo.Context):
        """
        Draw pattern on a cairo context in millimeter scale.

        Args:
            ctx: cairo context
        """


class ChArUcoPattern(CalibrationPattern):
    def __init__(self, width: int, height: int, square_size: float):
        """
        OpenCV style ChArUco pattern.

        Allows calibration with partly occluded pattern.

        Args:
            width: Number of squares in horizontal direction.
            height: Number of squares in vertical direction.
            square_size: Size of a square in millimeters.
        """
        self.width = width
        self.height = height
        self.square_size = square_size
        self.overall_size = (width * square_size, height * square_size)

        self.markersize = square_size / 8 * 6
        self._aruco = ArUco(4)

        self.charuco_board = self._aruco.create_charuco_board(
            width, height, square_size, self.markersize
        )

        objp = np.zeros(((width - 1) * (height - 1), 3), np.float32)
        objp[:, :2] = (
            (np.mgrid[0 : width - 1, 0 : height - 1] + 1) * square_size
        ).T.reshape(-1, 2)
        self.object_points = objp

        marker_margin = 0.5 * self.square_size - 0.5 * self.markersize

        even_row = np.arange(1, self.width, 2)
        odd_row = np.arange(0, self.width, 2)

        number_of_even_rows = self.height - self.height // 2
        number_of_odd_rows = self.height // 2

        self.marker_positions = np.empty(
            (
                len(even_row) * number_of_even_rows + len(odd_row) * number_of_odd_rows,
                2,
            ),
            dtype=np.float32,
        )
        stride = len(even_row) + len(odd_row)
        marker_margin = 0.5 * self.square_size - 0.5 * self.markersize
        for r in range(self.height):
            h = r // 2
            if r % 2 == 0:
                row = self.marker_positions[h * stride : h * stride + len(even_row)]
                row[..., 0] = even_row
            else:
                row = self.marker_positions[
                    h * stride + len(even_row) : h * stride + stride
                ]
                row[..., 0] = odd_row
            row[..., 1] = r
        self.marker_positions *= self.square_size
        self.marker_positions += marker_margin

        self.marker_ids = np.arange(len(self.marker_positions))

    def to_dict(self) -> dict:
        d = {
            "type": type(self).__name__,
            "width": self.width,
            "height": self.height,
            "square_size": self.square_size,
        }
        return d

    @classmethod
    def from_dict(cls, d: dict):
        return cls(d["width"], d["height"], d["square_size"])

    def draw_cairo(self, ctx: cairo.Context):
        # Draw chessboard squares
        s = self.square_size
        for i in range(self.width):
            for j in range(self.height):
                if (i + j) % 2 == 0:
                    ctx.move_to(i * s, j * s)
                    ctx.line_to((i + 1) * s, j * s)
                    ctx.line_to((i + 1) * s, (j + 1) * s)
                    ctx.line_to(i * s, (j + 1) * s)
                    ctx.fill()

        # Draw ArUco markers
        for marker_id, (x0, y0) in zip(self.marker_ids, self.marker_positions):
            ctx.save()
            ctx.translate(x0, y0)
            self._aruco.draw(ctx, marker_id, self.markersize)
            ctx.restore()

    def find_in_image(
        self, image: np.ndarray, plot: bool = False
    ) -> Tuple[np.ndarray, np.ndarray]:
        if image.ndim == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        all_corners, all_ids = self._aruco.detect(image)

        if all_ids is None or len(all_ids) < len(self.marker_positions) / 2:
            raise ValueError("Pattern not found")

        num_corners, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            all_corners, all_ids, image, board=self.charuco_board
        )

        if num_corners == 0:
            raise ValueError(
                "Pattern not found, but ArUco markers detected. "
                "Pattern config is likely to be wrong."
            )

        all_ids = all_ids.flatten()
        all_corners = np.asarray(all_corners)[:, 0]

        # check for illegal markers
        if len(set(all_ids) - set(self.marker_ids)):
            raise ValueError("Illegal aruco marker found. Pattern might not match.")

        # find accurate sub pixel corner positions
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        image_points_accurate = np.array(
            charuco_corners[:, 0], dtype=np.float32, copy=True, order="C"
        )
        cv2.cornerSubPix(image, image_points_accurate, (11, 11), (-1, -1), criteria)

        # CV2 has different pattern alignments depending on the version
        if _parse_version(cv2.__version__) < _parse_version("4.6"):
            # Flip object points vertically to align with aruco detection
            object_points = self.object_points.reshape(
                (self.height - 1, self.width - 1, 3)
            )
            object_points = object_points[::-1].reshape((-1, 3))[charuco_ids[:, 0]]
        else:
            object_points = self.object_points[charuco_ids[:, 0]]

        if plot:
            import matplotlib.pyplot as plt

            plt.figure(figsize=(15, 10))
            plt.imshow(image, cmap="gray")
            plt.plot(image_points_accurate[:, 0], image_points_accurate[:, 1], "x")
            plt.show()

        return image_points_accurate, object_points


class ArUco:
    def __init__(self, marker_pixel_size: int):
        """
        Utility to draw and detect ArUco markers.

        Args:
            marker_pixel_size: Number of pixels in the marker without the black border.
        """
        aruco_dict_def = {
            4: cv2.aruco.DICT_4X4_1000,
            5: cv2.aruco.DICT_5X5_1000,
            6: cv2.aruco.DICT_6X6_1000,
            7: cv2.aruco.DICT_7X7_1000,
        }[marker_pixel_size]

        self.marker_pixel_count = marker_pixel_size + 2

        try:
            # OpenCV < 4.7.x
            self.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_def)
        except AttributeError:
            # OpenCV >= 4.7.x
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_def)

    def draw(self, ctx: cairo.Context, marker_id: int, marker_size: float):
        c = self.marker_pixel_count

        ctx.save()
        ctx.scale(marker_size / c, marker_size / c)

        # Get ArUco marker bitmap
        try:
            # OpenCV < 4.7.x
            marker_image = cv2.aruco.drawMarker(self.aruco_dict, marker_id, c)
        except AttributeError:
            # OpenCV >= 4.7.x
            marker_image = cv2.aruco.generateImageMarker(self.aruco_dict, marker_id, c)

        # Extract and draw all black pixels
        for y, x in zip(*np.where(marker_image == 0)):
            ctx.rectangle(x.item(), y.item(), 1, 1)
            ctx.fill()
        ctx.restore()

    def detect(self, image: np.ndarray):
        try:
            # OpenCV < 4.7.x
            arucoParams = cv2.aruco.DetectorParameters_create()
            all_corners, all_ids, _ = cv2.aruco.detectMarkers(
                image, self.aruco_dict, parameters=arucoParams
            )
        except AttributeError:
            # OpenCV >= 4.7.x
            parameters = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(self.aruco_dict, parameters)
            all_corners, all_ids, _ = detector.detectMarkers(image)
        return np.asarray(all_corners), all_ids

    def create_charuco_board(
        self, width: int, height: int, square_size: float, marker_size: float
    ):
        try:
            # OpenCV < 4.7.x
            charuco_board = cv2.aruco.CharucoBoard_create(
                width, height, square_size, marker_size, self.aruco_dict
            )
        except AttributeError:
            # OpenCV >= 4.7.x
            charuco_board = cv2.aruco.CharucoBoard(
                (width, height), square_size, marker_size, self.aruco_dict
            )
        return charuco_board


@dataclasses.dataclass
class IntrinsicCalibration:
    """Intrinsic calibration data."""

    intrinsic: np.ndarray
    """3x3 intrinsic matrix."""
    dist_coeffs: Optional[np.ndarray]
    """Opencv distortion coefficients"""

    def cos_phi(self, imsize: Tuple[int, int], angle_margin_factor: float = 1) -> float:
        return float(
            np.cos(
                angle_margin_factor
                * np.arctan(np.linalg.norm(imsize) / self.intrinsic[0, 0] / 2)
            )
        )


@dataclasses.dataclass
class IntrinsicCalibrationWithData(IntrinsicCalibration):
    """Intrinsic calibration data with additional information."""

    image_size: Tuple[int, int]
    """Width and height in pixels."""
    rms_error: float
    """Root mean squared calibration error in pixels"""
    max_error: float
    """Maximum calibration error in pixels"""
    timestamp: Optional[int]
    """Unix timestamp in milliseconds"""
    image_points_batch: List[np.ndarray]
    """List of images point arrays in pixels [x, y]."""
    squared_error_distances: List[np.ndarray]
    """Squared reprojection error distance for all image points."""

    def plot(self, show: bool = False, save: Optional[Union[str, pathlib.Path]] = None):
        import matplotlib.pyplot as plt
        from scipy.interpolate import griddata

        pix = np.concatenate(self.image_points_batch, axis=0).reshape((-1, 2))
        error = np.sqrt(np.concatenate(self.squared_error_distances, axis=0).ravel())

        width, height = self.image_size

        grid_x, grid_y = np.mgrid[0:height, 0:width]
        error_image = griddata(pix, error, (grid_y, grid_x), method="linear").astype(
            np.float32
        )
        mask = np.isnan(error_image)

        factor = np.nanmax(error_image) / 255.0
        src = (np.where(mask, 0.0, error_image) / factor).astype(np.uint8)
        extrapolated = cv2.inpaint(
            src, mask.astype(np.uint8), 10, cv2.INPAINT_NS
        ).astype(np.float32)
        extrapolated *= factor
        error_image = np.where(mask, extrapolated, error_image)

        fig, ax = plt.subplots(figsize=(10, 6))
        im = ax.imshow(error_image)
        plt.colorbar(im, ax=ax)

        for pix in self.image_points_batch:
            ax.plot(pix[:, 0], pix[:, 1], "+", color="white")

        ax.set_title("Estimated euclidian error distance in pixels")

        if save is not None:
            fig.savefig(save)

        if show:
            fig.show()

        plt.close(fig)

    def plot_opencv(
        self,
        red_level: Optional[float] = None,
        green_level: Optional[float] = None,
        show_points: bool = True,
        show_legend: bool = True,
    ):
        """Plot calibration quality using OpenCV functions only."""

        w, h = self.image_size

        # Minimum and maximum levels for ample color scheme
        if red_level is None:
            red_level = np.minimum(h, w) / 10
        if green_level is None:
            green_level = 0.1

        red_level_log = np.log10(red_level)
        green_level_log = np.log10(green_level)
        mid_level = 10 ** ((red_level_log + green_level_log) * 0.5)

        # Average voronoi plots
        img = np.zeros((h, w), dtype=np.float32)
        for points, squared_errors in zip(
            self.image_points_batch, self.squared_error_distances
        ):
            intensities = np.sqrt(squared_errors)

            rect = (0, 0, w, h)
            subdiv = cv2.Subdiv2D(rect)
            subdiv.insert(points)

            img_voronoi = np.zeros((h, w), dtype=np.float32)
            facets, _ = subdiv.getVoronoiFacetList([])
            for poly, value in zip(facets, intensities):
                color = float(value)
                poly = poly.astype(int)
                cv2.fillConvexPoly(img_voronoi, poly, color, cv2.LINE_4, 0)
            img += img_voronoi

        # Convert image to log
        np.log10(img, out=img)

        if show_legend:
            img[-20:] = np.linspace(red_level_log, green_level_log, img.shape[1])

        # Map values to green yellow red color map
        img -= red_level_log
        img *= 120 / (green_level_log - red_level_log)

        np.clip(img, 0, 120, out=img)
        img = np.stack([img] + [np.ones_like(img)] * 2, axis=-1)
        cv2.cvtColor(img, cv2.COLOR_HSV2BGR, dst=img)
        img *= 255
        img = img.astype(np.uint8)

        if show_legend:
            color = (0, 0, 0)
            font = cv2.FONT_HERSHEY_SIMPLEX

            texts = {
                4: (f"{red_level:.2f}px", 0),
                w - 4: (f"{green_level:.2f}px", 1),
                w // 2: (f"{mid_level:.2f}px", 0.5),
            }

            for x, (text, pos) in texts.items():
                x -= int(pos * cv2.getTextSize(text, font, 0.5, 1)[0][0])
                cv2.putText(img, text, (x, h - 1 - 4), font, 0.5, color, 1, cv2.LINE_AA)

        # plot image points in black
        if show_points:
            color = (0, 0, 0)
            for points in np.concatenate(self.image_points_batch, axis=0).astype(int):
                cv2.circle(img, points, 2, color, cv2.FILLED, cv2.LINE_4, 0)

        return img


def calibration_from_image_files(
    image_files: Sequence[Union[pathlib.Path, str]],
    marker_pattern: CalibrationPattern,
    fraction: float = 1.0,
    verbose: bool = False,
    plot: bool = False,
) -> "IntrinsicCalibrationWithData":
    """Generate new calibration from image files."""
    if not image_files:
        raise ValueError("image_file must not be empty")

    def gen():
        for file in image_files:
            yield cv2.imread(str(file), flags=cv2.IMREAD_GRAYSCALE)

    return calibration_from_images(
        images=gen(),
        marker_pattern=marker_pattern,
        fraction=fraction,
        verbose=verbose,
    )


def calibration_from_images(
    images: Iterable[np.ndarray],
    marker_pattern: CalibrationPattern,
    fraction: float = 1.0,
    verbose: bool = False,
) -> "IntrinsicCalibrationWithData":
    """Generate new calibration from images and chessboard object."""

    # Arrays to store object points and image points from all the images.
    object_points_batch = []  # 3d point in real world space
    image_points_batch = []  # 2d points in image plane.

    image = None
    for i, image in enumerate(images):
        if image.ndim == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        try:
            image_points, object_points = marker_pattern.find_in_image(image)
            image_points_batch.append(image_points)
            object_points_batch.append(object_points)
            if verbose:
                print(f"Calibration pattern detected : {i}")
        except ValueError:
            if verbose:
                print(f"Calibration pattern not found : {i}")

    if image is None:
        raise ValueError("images may not be empty")

    if not len(image_points):
        raise ValueError("No points to calibrate found")

    height, width = image.shape[:2]

    return calibration_from_points(
        object_points_batch,
        image_points_batch,
        (width, height),
        fraction=fraction,
        verbose=verbose,
    )


def calibration_from_points(
    object_points_batch: Sequence,
    image_points_batch: Sequence,
    image_size: Tuple[int, int],
    fraction: float = 1.0,
    verbose: bool = False,
) -> "IntrinsicCalibrationWithData":
    """
    Generate new calibration from object points and image points

    Args:
        fraction (float): Fraction of Images used to calibrate.
            If fraction < 1, the calibration is performed twice where
            for the second time, only a fraction of the images is used.
    """

    if verbose:
        print("Calibrate ...")

    rms_error, intrinsic, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points_batch, image_points_batch, image_size, None, None
    )

    if fraction < 1:
        # test calibration accuracy
        inverted_scores = []
        for obj, pix, rvec, tvec in zip(
            object_points_batch, image_points_batch, rvecs, tvecs
        ):
            pix_calculated, _ = cv2.projectPoints(
                obj, rvec, tvec, intrinsic, dist_coeffs
            )
            inverted_score = np.sum((pix - pix_calculated[:, 0]) ** 2)
            inverted_scores.append(inverted_score)

        # set threshold to fraction percentile
        threshold_score = np.sort(inverted_scores)[int(len(inverted_scores) * fraction)]

        # pick images that are better than the threshold
        object_points_batch2 = []
        image_points_batch2 = []
        for obj, pix, inverted_score in zip(
            object_points_batch, image_points_batch, inverted_scores
        ):
            if inverted_score < threshold_score:
                object_points_batch2.append(obj)
                image_points_batch2.append(pix)
        object_points_batch = object_points_batch2
        image_points_batch = image_points_batch2

        if verbose:
            print(
                f"Calibrate again using better {int(fraction * 100)}th percentile ..."
            )

        # redo calibration with most accurate source images only
        rms_error, intrinsic, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            object_points_batch, image_points_batch, image_size, None, None
        )

    # test calibration accuracy
    squared_error_distances = []
    for obj, pix, rvec, tvec in zip(
        object_points_batch, image_points_batch, rvecs, tvecs
    ):
        pix_calculated, _ = cv2.projectPoints(obj, rvec, tvec, intrinsic, dist_coeffs)
        squared_error_distance = (
            np.linalg.norm(pix - pix_calculated[:, 0], axis=-1) ** 2
        )
        squared_error_distances.append(squared_error_distance)

    max_error = float(np.sqrt(np.max(np.concatenate(squared_error_distances, axis=0))))

    timestamp = int(time.time() * 1000)

    instance = IntrinsicCalibrationWithData(
        intrinsic,
        dist_coeffs,
        image_size,
        rms_error,
        max_error,
        timestamp,
        image_points_batch,
        squared_error_distances,
    )

    return instance


def look_at_extrinsic(
    target_pos: ArrayLike, eye_pos: ArrayLike, up_vector: ArrayLike
) -> np.ndarray:
    """Create a 4x4 extrinsic matrix of a camera positioned at eye_pos pointing to target_pos"""
    target_pos = np.asarray(target_pos, dtype=np.float64)
    eye_pos = np.asarray(eye_pos, dtype=np.float64)
    up_vector = np.asarray(up_vector, dtype=np.float64)

    f = target_pos - eye_pos
    s = np.cross(f, up_vector)

    m = np.eye(4)
    m[0, :3] = s
    m[1, :3] = np.cross(f, s)
    m[2, :3] = f
    m[:3, :3] /= np.linalg.norm(m[:3, :3], axis=1, keepdims=True)
    m[:3, 3] = m[:3, :3] @ -eye_pos

    return m


def intrinsic_from_camera_param(
    focal_length_mm: float, sensor_diagonal_mm: float, width: int, height: int
) -> np.ndarray:
    """Create an intrinsic matrix from camaera parameteres"""
    diagonal_pix = np.sqrt(width**2 + height**2)
    factor = sensor_diagonal_mm / diagonal_pix
    width_mm = width * factor

    f = focal_length_mm / width_mm * width
    tx = (width / 2) - 0.5
    ty = (height / 2) - 0.5

    intrinsic = np.array(
        (
            (f, 0, tx),
            (0, f, ty),
            (0, 0, 1),
        )
    )

    return intrinsic


class PointUndistorter:
    """Transform points between undistored image coordiantes and distorted image coordinates"""

    def __init__(self, cal):
        self.cal = cal

    def undisort(self, xy, axis=-1):
        """Transform points from distorted image coordiantes to undistorted image coordinates"""
        xy = np.asarray(xy, dtype=np.float32)
        xy = np.swapaxes(xy, axis, -1)
        shape = xy.shape
        xy = np.reshape(xy, (-1, 2))

        xy_undist = cv2.undistortPoints(
            xy, self.cal.intrinsic, self.cal.dist_coeffs, P=self.cal.intrinsic
        )[:, 0]

        xy_undist = np.reshape(xy_undist, shape)
        xy_undist = np.swapaxes(xy_undist, axis, -1)
        return xy_undist

    def distort(self, xy, axis=-1):
        """Transform points from undistorted image coordiantes to distorted image coordinates"""
        xy = np.asarray(xy, dtype=np.float32)
        xy = np.swapaxes(xy, axis, -1)
        shape = xy.shape
        xy = np.reshape(xy, (-1, 2))

        t = self.cal.intrinsic[:2, 2]
        f = np.diagonal(self.cal.intrinsic)[:2]
        k1, k2, p1, p2, k3 = self.cal.dist_coeffs.flatten()[:5]
        p12 = np.array((p1, p2))

        xy = (xy - t) / f

        r2 = np.sum(xy**2, axis=-1, keepdims=True)
        r4 = r2**2
        r6 = r4 * r2

        xy3 = xy * (1 + k1 * r2 + k2 * r4 + k3 * r6)
        xy_dist = (
            xy3
            + p12 * 2 * np.prod(xy, axis=-1, keepdims=True)
            + np.flip(p12) * (r2 + 2 * xy * xy)
        )
        xy_dist = (xy_dist * f) + t

        xy_dist = np.reshape(xy_dist, shape)
        xy_dist = np.swapaxes(xy_dist, axis, -1)
        return xy_dist


def decompose_cammat(cammat: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Decompose a 3x4 camera matrix to a 3x3 intrinsic and 3x4 extrinsic matrix
    """
    shape = cammat.shape[:-2]
    intrinsic = np.empty(shape + (3, 3), dtype=np.float64)
    extrinsic = np.empty(shape + (3, 4), dtype=np.float64)
    t = np.empty(shape + (4, 1), dtype=np.float64)
    cv2.decomposeProjectionMatrix(
        cammat, cameraMatrix=intrinsic, rotMatrix=extrinsic[..., :3, :3], transVect=t
    )
    extrinsic[:3, 3:] = extrinsic[:3, :3] @ (-t[:3] / t[3:4])

    return intrinsic, extrinsic


class PointProjector:
    """Object to project points from 3D space to 2D space."""

    camera_mat: np.ndarray
    """4x3 projection camera matrix."""
    intrinsic: np.ndarray
    """3x3 intrinsic camera matrix."""
    extrinsic: np.ndarray
    """4x3 extrinsic camera matrix."""
    dist_coeffs: Optional[np.ndarray]
    """Distortion coefficients or None."""

    def __init__(
        self,
        camera_mat: Optional[np.ndarray] = None,
        intrinsic: Optional[np.ndarray] = None,
        extrinsic: Optional[np.ndarray] = None,
        dist_coeffs: Optional[np.ndarray] = None,
    ):
        if camera_mat is None:
            if intrinsic is None or extrinsic is None:
                raise ValueError(
                    "Either camera_mat or intrinsic and extrinsic must be given"
                )
            self.camera_mat = intrinsic[:3, :3] @ extrinsic[:3, :4]
        else:
            self.camera_mat = camera_mat

        self.camera_mat_inverse = np.linalg.inv(self.camera_mat[:, :3])
        self._b = (self.camera_mat_inverse[2] @ self.camera_mat[:, 3:])[0]
        if intrinsic is None or extrinsic is None:
            self.intrinsic, self.extrinsic = decompose_cammat(self.camera_mat)
        else:
            self.intrinsic = intrinsic[:3, :3]
            self.extrinsic = extrinsic[:3, :4]

        self.dist_coeffs = dist_coeffs
        if dist_coeffs is not None:
            self._undistorter = PointUndistorter(
                IntrinsicCalibration(self.intrinsic, self.dist_coeffs)
            )

    def pix2obj(
        self, xy: ArrayLike, Z: Union[ArrayLike, float] = 0.0, axis: int = -1
    ) -> np.ndarray:
        """
        Convert pixel coordiantes to object coordinates

        Args:
            xy (np.ndarray): pixel coordinates, shape=[2, n]
            Z (np.ndarray or int, optional): Z object coordinate. Defaults to 0.
            axis (int): axis of the XYZ array that represents the 2-dimensional part

        Returns:
            np.ndarray: object coordinates, shape=[3, n]
        """
        xy = np.swapaxes(xy, axis, -1)[..., None]
        if self._undistorter is not None:
            xy = self._undistorter.undisort(xy, axis=-1)
        Z = np.asarray(Z)
        if Z.ndim != 0:
            Z = np.swapaxes(Z, axis, -1)
        xy1 = np.concatenate((xy, np.ones_like(xy[..., :1, :])), axis=-2)
        s = (Z + self._b) / (self.camera_mat_inverse[2] @ xy1)
        XYZ = self.camera_mat_inverse @ (s[..., None] * xy1 - self.camera_mat[:, 3:])
        return np.swapaxes(XYZ[..., 0], axis, -1)

    def obj2pix(self, XYZ: ArrayLike, axis: int = -1) -> np.ndarray:
        """
        Convert object coordinates to undistorted pixel coordinates

        Args:
            XYZ (np.ndarray): object coordinates, shape=[3, n]
            axis (int): axis of the XYZ array that represents the 3-dimensional part

        Returns:
            np.ndarray: undistorted pixel coordinates, shape=[2, 2]
        """
        XYZ = np.swapaxes(XYZ, axis, -1)[..., None]
        pix_xys = self.camera_mat[:, :3] @ XYZ + self.camera_mat[:, 3:]
        pix_xy = pix_xys[..., :2, 0] / pix_xys[..., 2:, 0]
        if self._undistorter is not None:
            pix_xy = self._undistorter.distort(pix_xy, axis=-1)
        pix_xy = np.swapaxes(pix_xy, axis, -1)
        return pix_xy
