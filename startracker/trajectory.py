import dataclasses

import numpy as np
import matplotlib.pyplot as plt
import scipy.spatial.transform


def fermat_point(points: np.ndarray, plot: bool = False) -> np.ndarray:
    """
    Solves the fermat point problem.

    Finds the point, which has equal angles to all vertices (120 degrees).

    https://en.wikipedia.org/wiki/Fermat_point

    Args:
        points: point array with shape=[..., 3, xyz]

    Returns:
        (np.ndarray): fermat mid point shape=[..., xyz]
    """
    a, b, c = np.moveaxis(points, -2, 0)

    ba = a - b
    bc = c - b
    normal = np.cross(bc, ba)

    vec1 = np.cross(normal, ba)
    vec1 *= (np.sqrt(3) / 2) * (
        np.linalg.norm(ba, axis=-1, keepdims=True)
        / np.linalg.norm(vec1, axis=-1, keepdims=True)
    )
    q = b + (ba / 2) + vec1

    vec2 = np.cross(bc, normal)
    vec2 *= (np.sqrt(3) / 2) * (
        np.linalg.norm(bc, axis=-1, keepdims=True)
        / np.linalg.norm(vec2, axis=-1, keepdims=True)
    )
    r = b + (bc / 2) + vec2

    A = np.zeros(a.shape[:-1] + (6, 5))
    A[..., :3, 0] = r - a
    A[..., 3:, 1] = q - c
    A[..., :3, -3:] = np.eye(3)
    A[..., 3:, -3:] = np.eye(3)
    B = np.concatenate((a, c), axis=-1)[..., None]

    res = np.linalg.solve(A[..., :5, :], B[..., :5, :])
    m = res[..., -3:, 0]

    if plot and a.ndim == 1:
        import matplotlib.pyplot as plt

        ax = plt.figure().add_subplot(projection="3d")

        for n, (x, y, z) in zip("abcqrm", [a, b, c, q, r, m]):
            ax.plot(x, y, z, ".", color="black", label="motor positions")
            ax.text(x, y, z, n)

        plt.axis("equal")
        plt.show()

    return m


class MotorSolver:
    MOTOR_COUNT = 3

    def __init__(
        self,
        motor_dists_from_center: float = 50,
        theta: float = np.pi / 4,
        phi: float = 1.0,
    ):
        """
        Initialize.

        Args:
            motor_dists_from_center: Distance between the motor shaft start points and the center.
            theta: Azimuth angle of the motor shaft relative to the vector to the center.
            phi: Elevation angle of the motor shaft upward relative to the motor plane.
        """
        MOTOR_COUNT = self.MOTOR_COUNT

        angles = np.arange(MOTOR_COUNT) / MOTOR_COUNT * (np.pi * 2)
        self.motor_vectors = np.stack(
            (np.cos(angles), np.sin(angles), np.zeros_like(angles)), axis=-1
        )

        motor_vectors_perp = np.stack(
            (-np.sin(angles), np.cos(angles), np.zeros_like(angles)), axis=-1
        )

        self.motor_positions = self.motor_vectors * motor_dists_from_center
        shaft_directions = (
            np.cos(theta) * self.motor_vectors + np.sin(theta) * motor_vectors_perp
        ) * np.cos(phi) + [0, 0, np.sin(phi)]
        shaft_directions /= np.linalg.norm(shaft_directions, axis=-1)
        self.shaft_directions = shaft_directions

    def solve_motor_dists(self, rot_matrix, plot: bool = False) -> np.ndarray:
        """
        Solve the distances of the linear motors.

        Supports batching.

        Args:
            rotm_matrix: Rotation matrix shape=[..., 3, 3].
            plot: show matplotlib of the motor alignment.

        Returns:
            np.ndarray: linear motor distances shape=[..., 3 (motor), 3 (xyz)]
        """
        motor_positions = self.motor_positions
        shaft_directions = self.shaft_directions

        transformed_vectors = np.swapaxes(
            rot_matrix @ np.swapaxes(self.motor_vectors, -1, -2), -1, -2
        )
        # transformed_vectors = (self.motor_vectors @ rot_matrix)

        batch_shape = rot_matrix.shape[:-2]

        A = np.zeros(batch_shape + (9, 9))
        for i in range(self.MOTOR_COUNT):
            A[..., i * 3 : (i + 1) * 3, :3] = np.eye(3)
            A[..., i * 3 : (i + 1) * 3, 3 + i] = transformed_vectors[..., i, :]
            A[..., i * 3 : (i + 1) * 3, 6 + i] = -shaft_directions[i]
        b = np.broadcast_to(
            motor_positions.ravel(), (1,) * (rot_matrix.ndim - 2) + (9,)
        )

        assert np.all(np.linalg.det(A) != 0)

        solution = np.linalg.solve(A, b)

        motor_dists = solution[..., -3:]
        sled_dists = solution[..., 3:6]

        # assert np.all(sled_dists > 0)

        if (not np.all(sled_dists > 0)) or (plot and motor_dists.ndim == 1):
            translation = solution[:3]
            shaft_vectors = shaft_directions * motor_dists[:, None]

            ax = plt.figure().add_subplot(projection="3d")

            for i in range(self.MOTOR_COUNT):
                x, y, z = np.stack((motor_positions[i], np.zeros(3)), axis=1)
                ax.plot(x, y, z, ".-", color="black", label="motor positions")

                x, y, z = np.stack(
                    (motor_positions[i], motor_positions[i] + shaft_directions[i] * 10),
                    axis=1,
                )
                ax.plot(x, y, z, color="red", label="shaft", lw=1)

                x, y, z = np.stack(
                    (motor_positions[i], motor_positions[i] + shaft_vectors[i]), axis=1
                )
                ax.plot(x, y, z, color="blue", label="shaft")

                x, y, z = np.stack(
                    (translation, motor_positions[i] + shaft_vectors[i]), axis=1
                )
                ax.plot(x, y, z, color="green", label="shaft")

                x, y, z = np.stack(
                    (translation, translation + transformed_vectors[i] * 10), axis=1
                )
                ax.plot(x, y, z, color="orange", label="shaft")

            x, y, z = np.stack((np.zeros(3), translation), axis=1)
            ax.plot(x, y, z, label="tranlation")

            plt.axis("equal")
            plt.show()

        return motor_dists

    def get_rotm(self, motor_dists: np.ndarray) -> np.ndarray:
        """
        Solve the rotation matrix from linear motor distances.

        Supports batching.

        Args:
            motor_dists: Linear motor distances shape=[..., 3 (motor), 3 (xyz)]

        Returns:
            np.ndarray: Rotation matrix shape=[..., 3, 3].
        """
        motor_positions = self.motor_positions
        shaft_directions = self.shaft_directions

        # calculate pivot points
        pivots = motor_positions + shaft_directions * motor_dists[..., None]

        # Find mid point that is angled 120 to all pivots
        mid_point = fermat_point(pivots)

        # Calculate target vector and normal
        target = pivots[..., 0, :] - mid_point
        a, b, c = np.moveaxis(pivots, -2, 0)
        normal = np.cross(c - b, a - b)

        # Build orthogonal vectors using target and normal vector
        x = target
        x /= np.linalg.norm(x, axis=-1, keepdims=True)
        z = normal
        z /= np.linalg.norm(z, axis=-1, keepdims=True)
        y = np.cross(z, x)

        # Construct rotation matrix
        rot_matrix = np.stack((x, y, z), axis=-1)

        return rot_matrix


def astro_rotation_matrix(
    azimuth: float,
    elevation: float,
    roll: float,
    degrees: bool = False,
):
    """
    Get rotation matrix from azimuth and elevation parameters.

    Args:
        azimuth: Angle around zenith from x-axis.
        elevation: Angle from ground plane to zenith.
        rol: Right hand angle around direction vector.
        degrees: Use degrees instead of rads

    Returns:
        np.ndarray: Rotation matrix shape=[3,3]
    """
    az_el_axis = (
        scipy.spatial.transform.Rotation.from_euler(
            "yzx", [-elevation, azimuth, 0.0], degrees=degrees
        ).as_matrix()
        @ np.array((1, 0, 0))[:, None]
    )[:, 0]

    roll = np.asarray(roll)[..., None]

    rot_matrix = scipy.spatial.transform.Rotation.from_rotvec(
        az_el_axis * roll, degrees=degrees
    ).as_matrix()
    return rot_matrix


def seconds_to_degrees(seconds: float):
    return seconds * (360 / (24 * 60 * 60))

def degrees_to_seconds(degrees: float):
    return degrees / (360 / (24 * 60 * 60))


@dataclasses.dataclass
class PolynomTrajectory:
    position_coeffs: np.ndarray
    start: float
    stop: float


def find_continuous_zero_slice(x: np.ndarray, mid_index: int) -> slice:
    assert 0 <= mid_index <= len(x)
    start_index = mid_index - x[mid_index::-1].argmax().item()
    if x[start_index] == 0:
        start_index = 0
    else:
        start_index += 1
    stop_index = mid_index + x[mid_index:].argmax().item()
    if x[stop_index] == 0:
        stop_index = len(x)
    return slice(start_index, stop_index)


class TrajectoryCalculator:
    def __init__(
        self,
        max_seconds: float,
        max_dist: float,
        motor_solver: MotorSolver,
    ):
        self._max_seconds = max_seconds
        self._max_dist = max_dist
        self._motor_solver = motor_solver

    def __call__(self, azimuth: float, elevation: float):
        seconds = np.linspace(-self._max_seconds / 2, self._max_seconds / 2, 100)
        roll = seconds_to_degrees(seconds)

        rot_matrices = astro_rotation_matrix(azimuth, elevation, roll, degrees=True)

        print(rot_matrices.shape)

        motor_dists = self._motor_solver.solve_motor_dists(rot_matrices)

        dists_exceeds_neg_bound = motor_dists.min(axis=-1) < -self._max_dist
        dists_exceeds_pos_bound = motor_dists.max(axis=-1) > self._max_dist

        slc = find_continuous_zero_slice(
            dists_exceeds_neg_bound + dists_exceeds_pos_bound, len(motor_dists) // 2
        )

        motor_dists = motor_dists[slc]
        seconds = seconds[slc]

        if len(seconds) == 0:
            raise ValueError("Calculated trajectory has zero length")

        poly = np.polyfit(seconds, motor_dists, 3)

        return PolynomTrajectory(
            position_coeffs=poly,
            start=seconds.min().item(),
            stop=seconds.max().item(),
        )
