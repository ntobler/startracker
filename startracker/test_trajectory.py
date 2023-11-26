import numpy as np
import scipy.spatial.transform

from . import trajectory


def test_fermat_point():
    def _test_fermat_point(points):
        m = trajectory.fermat_point(points, plot=False)

        vecs = points - m[..., None, :]
        vecs /= np.linalg.norm(vecs, axis=-1, keepdims=True)

        dotproducts = np.sum(vecs * np.roll(vecs, 1, axis=-2), axis=-1)
        assert np.allclose(np.abs(dotproducts), 0.5)

    rng = np.random.default_rng(42)
    _test_fermat_point(rng.normal(size=(3, 3)))
    _test_fermat_point(rng.normal(size=(10, 3, 3)))
    _test_fermat_point(rng.normal(size=(7, 4, 3, 3)))


def test_motor_solver():
    def _test(rot_matrix):
        m = trajectory.MotorSolver()
        assert np.allclose(m.get_rotm(m.solve_motor_dists(rot_matrix)), rot_matrix)

    rng = np.random.default_rng(42)
    desired_rot = scipy.spatial.transform.Rotation.from_euler(
        "XYZ", [2, 1, 1], degrees=True
    )
    _test(desired_rot.as_matrix())
    desired_rot = scipy.spatial.transform.Rotation.from_euler(
        "XYZ", rng.normal(size=(100, 3)), degrees=True
    )
    _test(desired_rot.as_matrix())
    _test(desired_rot.as_matrix().reshape((10, 10, 3, 3)))


def test_find_continuous_zero_slice():
    assert trajectory.find_continuous_zero_slice(
        np.array([1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1], dtype=bool), 5
    ) == slice(4, 10)
    assert trajectory.find_continuous_zero_slice(
        np.array([0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1], dtype=bool), 5
    ) == slice(0, 7)
    assert trajectory.find_continuous_zero_slice(
        np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=bool), 5
    ) == slice(0, 10)
    assert trajectory.find_continuous_zero_slice(
        np.array([1, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=bool), 5
    ) == slice(1, 10)
    assert trajectory.find_continuous_zero_slice(
        np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 1], dtype=bool), 5
    ) == slice(0, 9)
    assert trajectory.find_continuous_zero_slice(
        np.array([0, 0, 0, 0, 0, 0, 1, 1, 1, 1], dtype=bool), 5
    ) == slice(0, 6)
    assert trajectory.find_continuous_zero_slice(
        np.array([1, 1, 1, 1, 1, 0, 1, 1, 1, 1], dtype=bool), 5
    ) == slice(5, 6)
    assert trajectory.find_continuous_zero_slice(
        np.array([1, 1, 1, 1, 1, 0, 0, 0, 0, 0], dtype=bool), 5
    ) == slice(5, 10)
    assert trajectory.find_continuous_zero_slice(
        np.array([1, 1, 1, 1, 1, 1, 1, 1, 1, 1], dtype=bool), 5
    ) == slice(6, 5)
