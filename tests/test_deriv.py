import numpy as np
import pytest
import scipy.spatial
import sympy as sp

from startracker import calibration, deriv, kalkam


def test_expand_pow_manual():
    x = sp.symbols("x")
    assert str(deriv.reformat_expr(x**5)) == "x*x*x*x*x"
    assert str(deriv.reformat_expr(2.0 * x**5)) == "2.0*(x*x*x*x*x)"
    assert str(deriv.reformat_expr(x**-5)) == "1.0/((x*x*x*x*x))"
    assert str(deriv.reformat_expr(2.0 * x**-5)) == "2.0*(1.0/((x*x*x*x*x)))"


def test_mrp():
    mrp_0, mrp_1, mrp_2 = sp.symbols("mrp_0 mrp_1 mrp_2")
    p = sp.Matrix([[mrp_0], [mrp_1], [mrp_2]])

    # Evaluate extrinsic with given parameter values
    mrp = [0.1, 0.2, 0.3]
    param_values = {mrp_0: mrp[0], mrp_1: mrp[1], mrp_2: mrp[2]}
    a = np.array(deriv.mrp_to_rotm(p).evalf(subs=param_values).tolist(), dtype=np.float64)
    b = scipy.spatial.transform.Rotation.from_mrp(mrp).as_matrix()
    np.testing.assert_allclose(a, b.T)

    mrp_inv = [-x for x in mrp]
    c = scipy.spatial.transform.Rotation.from_mrp(mrp_inv).as_matrix()
    np.testing.assert_allclose(a, c)


def test_distortion():
    cal = calibration.make_dummy()
    intrinsic = cal.intrinsic
    width, height = cal.image_size
    assert cal.dist_coeffs is not None
    cal.dist_coeffs[1:] = 0  # Only r^2 doefficient is handles

    dist = cal.dist_coeffs[0]
    rng = np.random.default_rng(42)
    points = rng.uniform(0, 1, size=(20, 2)) * [width, height]

    params = sp.symbols("fx tx fy ty k1")
    fx, tx, fy, ty, k1 = params

    # Image coordinates in pixel space
    img_x, img_y = sp.symbols("img_x img_y")

    subs = {
        fx: intrinsic[0, 0],
        tx: intrinsic[0, 2],
        fy: intrinsic[1, 1],
        ty: intrinsic[1, 2],
        k1: dist,
    }

    img_xy = sp.Matrix([[img_x], [img_y]])
    intrinsic = sp.Matrix([[fx, 0, tx], [0, fy, ty], [0, 0, 1]])

    undistorted, _ = deriv.undistort(img_xy, k1, intrinsic)

    for x, y in points:
        subs[img_x] = x
        subs[img_y] = y

        value = np.array(undistorted.subs(subs).evalf().tolist(), dtype=np.float64)
        value_gt = kalkam.PointUndistorter(cal).undisort((x, y))[:, np.newaxis]

        np.testing.assert_allclose(value, value_gt, atol=0.03)


if __name__ == "__main__":
    pytest.main([__file__])
