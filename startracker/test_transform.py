import numpy as np

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
