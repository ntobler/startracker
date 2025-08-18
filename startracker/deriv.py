"""Script to compute the Jacobian of the star tracker model with respect to its parameters.

Used to generate the optimized code in the starcal.rs and stargradcal.rs file.
"""

import sympy as sp  # type: ignore[import-untyped]


def reformat_expr(expr):
    """Reformat Sympy expressions, so they are compatible with rust code."""
    if isinstance(expr, sp.Pow) and expr.exp.is_Integer:
        base = reformat_expr(expr.base)
        exp = expr.exp
        if exp > 0:
            return sp.Mul(*[base] * exp, evaluate=False)
        divisor = sp.Pow(sp.Mul(*[base] * (-exp), evaluate=False), -1, evaluate=False)
        return sp.Mul(1.0, divisor, evaluate=False)
    if isinstance(expr, sp.Integer):
        return sp.Float(expr)
    if expr.args:
        return expr.func(*[reformat_expr(arg) for arg in expr.args], evaluate=False)
    return expr


def print_rust_code(values: list[sp.Expr], name="Values"):
    """Print expressions as rust code."""
    replacements, reduced_exprs = sp.cse(values)
    # Output the reusable subexpressions and final simplified derivatives
    print("// Reusable subexpressions:")
    for lhs, expr in replacements:
        # expr = sp.ratsimp(expr)
        expr = sp.nsimplify(expr, rational=True)
        print(f"let {lhs} = {reformat_expr(expr)};")
    print(f"\n// {name}:")
    for i, expr in enumerate(reduced_exprs):
        # expr = sp.ratsimp(expr)
        expr = sp.nsimplify(expr, rational=True)
        print(f"{name.lower()}[{i}] = {reformat_expr(expr)};")


def mrp_to_rotm(p: sp.Matrix) -> sp.Matrix:
    """Create rotation matrix from Modified Rodrigues Parameters (MRPs).

    Note: This is the inverse of the MRP representation of scipy.spatial.transform.Rotation

    Args:
        p: 3x1 sympy Matrix (Modified Rodrigues vector: p = u * tan(anlgle / 4))

    Returns:
        3x1 sympy Matrix (the rotated vector)
    """
    # Norm squared of p
    p_norm2 = p.dot(p)

    # Cross-product matrix [p]_x
    p_cross = sp.Matrix([[0, -p[2], p[1]], [p[2], 0, -p[0]], [-p[1], p[0], 0]])

    # Rotation matrix using MRPs
    numerator = 8 * p_cross * p_cross - 4 * (1 - p_norm2) * p_cross
    return sp.eye(3) + numerator / (1 + p_norm2) ** 2


def optim1():
    """Optimization problem used in starcal.rs."""
    # Define all symbols
    # Intrinsic camera parameters
    mrp_0, mrp_1, mrp_2 = sp.symbols("mrp_0 mrp_1 mrp_2")
    fx, tx, fy, ty = sp.symbols("fx tx fy ty")
    k1, k2, p1, p2, k3 = sp.symbols("k1 k2 p1 p2 k3")

    params = [mrp_0, mrp_1, mrp_2, fx, tx, fy, ty, k1, k2, p1, p2, k3]

    # Object coordinates in the camera frame
    obj_x, obj_y, obj_z = sp.symbols("obj_x obj_y obj_z")
    # Image coordinates in pixel space
    img_x, img_y = sp.symbols("img_x img_y")

    intrinsic = sp.Matrix([[fx, 0, tx], [0, fy, ty], [0, 0, 1]])
    obj_xyz = sp.Matrix([obj_x, obj_y, obj_z])
    rot = sp.Matrix([mrp_0, mrp_1, mrp_2])

    obj_xyz_rot = mrp_to_rotm(rot) * obj_xyz

    temp = intrinsic * obj_xyz_rot
    x = temp[0] / temp[2]
    y = temp[1] / temp[2]

    x = (x - tx) / fx
    y = (y - ty) / fy

    r2 = x**2 + y**2
    r4 = r2 * r2
    r6 = r2 * r4
    d = 1 + k1 * r2 + k2 * r4 + k3 * r6

    x_dist = x * d + (2 * p1 * x * y + p2 * (r2 + 2 * x**2))
    y_dist = y * d + (2 * p2 * x * y + p1 * (r2 + 2 * y**2))

    x_dist = (x_dist * fx) + tx
    y_dist = (y_dist * fy) + ty

    rx = img_x - x_dist
    ry = img_y - y_dist

    residuals = [rx, ry]
    jacobian = [sp.diff(rx, p) for p in params] + [sp.diff(ry, p) for p in params]

    print_rust_code(residuals, "Residual")
    print_rust_code(jacobian, "Jacobian")


def undistort(xy_dist: sp.Matrix, dist_coef: sp.Symbol, intrinsic: sp.Matrix) -> sp.Matrix:
    """Undistort image points with a first order radial barrel distortion."""
    fx = intrinsic[0, 0]
    tx = intrinsic[0, 2]
    fy = intrinsic[1, 1]
    ty = intrinsic[1, 2]
    k1 = dist_coef

    xy_dist = sp.Matrix([[(xy_dist[0, 0] - tx) / fx], [(xy_dist[1, 0] - ty) / fy]])
    d_xy_dist = sp.Matrix([[1 / fx, 0], [0, 1 / fy]])

    # only possible with iterative algorithm
    xy = xy_dist
    d_xy = d_xy_dist
    for _ in range(2):
        r2 = (xy.T * xy)[0, 0]
        d_r2 = 2 * d_xy @ xy
        xy = xy_dist / (1 + k1 * r2)
        d_xy = ((d_xy_dist * (1 + k1 * r2)) - (xy_dist @ (k1 * d_r2).T)) / (1 + k1 * r2) ** 2
    xy = sp.Matrix([[(xy[0, 0] * fx) + tx], [(xy[1, 0] * fy) + ty]])
    d_xy = sp.Matrix([[fx, 0], [0, fy]]) * d_xy

    return xy, d_xy


STAR_MOVEMENT_TRANSFORM = sp.Matrix(
    [
        [0, 1.0, 0],
        [-1.0, 0, 0],
        [0, 0, 0],
    ]
)


def project(mat: sp.Matrix, x: sp.Matrix) -> tuple[sp.Matrix, sp.Matrix]:
    """Project 3D point to 2D using a camera matrix.

    Args:
        mat: Camera matrix
        x: 3d point vector

    Return:
        Value and gradient.
    """
    f = mat[:2, :3]
    g = mat[2:, :3]
    gx = (g * x)[0, 0]
    fx = f * x
    y = fx / gx
    dy_dx = (f * gx - fx * g) / (gx * gx)
    return y, dy_dx


def optim2():
    """Optimization problem used in stargradcal.rs."""
    # Define all symbols
    params = sp.symbols("m_epsilon m_theta fx tx fy ty k1")
    m_epsilon, m_theta, fx, tx, fy, ty, k1 = params

    # Image coordinates in pixel space
    img_x, img_y = sp.symbols("img_x img_y")
    # Temporal gradient of image coordinates in pixel space
    img_dx, img_dy = sp.symbols("img_dx img_dy")

    img_xy = sp.Matrix([[img_x], [img_y]])
    img_dxy = sp.Matrix([[img_dx], [img_dy]])
    intrinsic = sp.Matrix([[fx, 0, tx], [0, fy, ty], [0, 0, 1]])
    intrinsic_inv = intrinsic.inv()

    # Build rotation matrix
    rot = (
        sp.Matrix([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
        * mrp_to_rotm(sp.Matrix([m_theta, 0, 0]))
        * mrp_to_rotm(sp.Matrix([0, 0, -m_epsilon]))
    )
    extrinsic = rot.T

    # Correct distortion
    stars_xy, d_undistort = undistort(img_xy, k1, intrinsic)
    stars_dxy = d_undistort * img_dxy

    # Transform image coordinates to normalized 3d coordinates
    stars_xyz = sp.Matrix([[stars_xy[0, 0]], [stars_xy[1, 0]], [1]])
    object_space = (extrinsic.T @ intrinsic_inv) @ stars_xyz
    object_space = object_space / sp.sqrt(object_space.T * object_space)[0, 0]

    # Calculate movement of stars in 3D
    star_movement_3d = STAR_MOVEMENT_TRANSFORM * object_space

    # Calculate movement of stars in 2D
    _, derivative = project(intrinsic * extrinsic, object_space)
    star_dir = derivative * star_movement_3d

    residuals = star_dir - stars_dxy
    rx = residuals[0, 0]
    ry = residuals[1, 0]

    residuals = [rx, ry]
    jacobian = [sp.diff(rx, p) for p in params] + [sp.diff(ry, p) for p in params]

    print_rust_code(residuals, "Residual")
    print_rust_code(jacobian, "Jacobian")


if __name__ == "__main__":
    optim1()
    optim2()
