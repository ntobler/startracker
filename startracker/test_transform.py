import pytest

import numpy as np
import scipy.spatial.transform

from . import transform


def test_azel_nwu():
    rng = np.random.default_rng(42)

    nwu = rng.normal(size=(10, 10, 3))
    nwu /= np.linalg.norm(nwu, axis=-1, keepdims=True)

    res = transform.azel2nwu(transform.nwu2azel(nwu))
    res /= np.linalg.norm(res, axis=-1, keepdims=True)

    print(np.allclose(nwu[..., 0], res[..., 0]))
    print(np.allclose(nwu[..., 1], res[..., 1]))
    print(np.allclose(nwu[..., 2], res[..., 2]))
    assert np.allclose(nwu, res)


@pytest.mark.parametrize(
    "func",
    [transform.find_common_rotation_axis, transform.find_common_rotation_axis_alt],
)
def test_find_common_rotation_axis(func):
    N = 128

    rng = np.random.default_rng(42)

    quat = rng.normal(size=4)
    quat /= np.linalg.norm(quat)
    origin_rot = scipy.spatial.transform.Rotation.from_quat(quat)

    rot_axis = rng.normal(size=3)
    rot_axis /= np.linalg.norm(rot_axis)

    rotvecs = rot_axis[None] * rng.uniform(0, np.pi * 2, size=(N, 1))
    around_axis_rot = scipy.spatial.transform.Rotation.from_rotvec(
        rotvecs, degrees=False
    )

    rots = (origin_rot * around_axis_rot).as_quat()

    axis, std = func(rots)

    assert np.allclose(rot_axis, axis, rtol=1e-4) or np.allclose(
        -rot_axis, axis, rtol=1e-4
    )

    rots += rng.normal(size=rots.shape) * 0.001
    rots /= np.linalg.norm(rots, axis=-1, keepdims=True)

    axis, std = func(rots)

    assert std < 0.01


@pytest.mark.parametrize(
    "func",
    [transform.find_common_rotation_axis, transform.find_common_rotation_axis_alt],
)
def test_find_common_rotation_axis_convergence(func):
    N = 10
    STD = 0.1

    rng = np.random.default_rng(43)

    quat = rng.normal(size=4)
    quat /= np.linalg.norm(quat)
    origin_rot = scipy.spatial.transform.Rotation.from_quat(quat)

    rot_axis = rng.normal(size=3)
    rot_axis /= np.linalg.norm(rot_axis)

    numbers = np.logspace(np.log10(3), np.log10(300), N).astype(int)
    errors = []
    estimates = []

    for i in numbers:
        rotvecs = rot_axis[None] * rng.uniform(0, np.pi * 2, size=(i, 1))
        around_axis_rot = scipy.spatial.transform.Rotation.from_rotvec(
            rotvecs, degrees=False
        )
        rots = (origin_rot * around_axis_rot).as_quat()

        rots += rng.normal(size=rots.shape) * STD
        rots /= np.linalg.norm(rots, axis=-1, keepdims=True)

        est, error = func(rots)
        estimates.append(est)
        errors.append(error)

    estimates = np.array(estimates)

    estimates = np.where(
        np.sum(estimates * rot_axis, axis=-1, keepdims=True) > 0, estimates, -estimates
    )

    true_errors = np.arccos(np.sum(estimates * rot_axis, axis=-1))

    errors_poly = np.polyfit(np.log10(numbers), np.log10(errors), 1)
    true_errors_poly = np.polyfit(np.log10(numbers), np.log10(true_errors), 1)

    if False:
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots()
        fig.suptitle("Convergence of error")
        (p1,) = ax.loglog(numbers, errors, ".-", label="algorithm estimated error")
        (p2,) = ax.loglog(numbers, true_errors, ".-", label="true error")
        ax.set_ylabel("Error (rad)")
        ax.set_xlabel("Number of samples")

        n = np.array([1, 10, 100, 1000])
        v = np.pi * STD / np.sqrt(n)
        ax.loglog(n, v, "--", label="natural std propagation")
        ax.loglog(
            n, 10 ** np.polyval(errors_poly, np.log10(n)), "--", color=p1.get_color()
        )
        ax.loglog(
            n,
            10 ** np.polyval(true_errors_poly, np.log10(n)),
            "--",
            color=p2.get_color(),
        )

        ax.grid()
        ax.legend()
        plt.show()

    # check if error approximately improves with sqrt n
    np.testing.assert_allclose(errors_poly[0], -0.5, rtol=0.2)
    np.testing.assert_allclose(true_errors_poly[0], -0.5, rtol=0.2)
