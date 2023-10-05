import numpy as np
import scipy.spatial.transform

import trajectory


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
