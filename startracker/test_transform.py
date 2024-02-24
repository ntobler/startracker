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


def test_find_common_rotation_axis():

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

    axis, std = transform.find_common_rotation_axis(rots)

    assert np.allclose(rot_axis, axis) or np.allclose(-rot_axis, axis)

    rots += rng.normal(size=rots.shape) * 0.001
    rots /= np.linalg.norm(rots, axis=-1, keepdims=True)

    axis, std = transform.find_common_rotation_axis(rots)

    assert std < 0.01
