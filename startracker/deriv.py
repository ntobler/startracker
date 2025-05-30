"""Script to compute the Jacobian of the star tracker model with respect to its parameters.

Used to generate the optimized code in the starcal.rs file.
"""

import sympy as sp


def mrp_rotate(v_input: sp.Matrix, p: sp.Matrix):
    """Rotate a 3D vector using Modified Rodrigues Parameters (MRPs).

    Args:
        v_input: 3x1 sympy Matrix (the input 3D vector)
        p: 3x1 sympy Matrix (Modified Rodrigues vector: p = u * tan(θ / 4))

    Returns:
        3x1 sympy Matrix (the rotated vector)
    """
    # Norm squared of p
    p_norm2 = p.dot(p)

    # Cross-product matrix [p]_x
    p_cross = sp.Matrix([[0, -p[2], p[1]], [p[2], 0, -p[0]], [-p[1], p[0], 0]])

    # Rotation matrix using MRPs
    numerator = 8 * p_cross * p_cross - 4 * (1 - p_norm2) * p_cross
    rotm = sp.eye(3) + numerator / (1 + p_norm2) ** 2

    return rotm * v_input


# Define all symbols
# Intrinsic camera parameters
mrp_0, mrp_1, mrp_2 = sp.symbols(
    "mrp_0 mrp_1 mrp_2"
)  # Rodrigues vector (axis-angle representation)
fx, sh, tx, fy, ty = sp.symbols("fx sh tx fy ty")
k1, k2, p1, p2, k3 = sp.symbols("k1 k2 p1 p2 k3")

params = [mrp_0, mrp_1, mrp_2, fx, sh, tx, fy, ty, k1, k2, p1, p2, k3]


# Object coordinates in the camera frame
obj_x, obj_y, obj_z = sp.symbols("obj_x obj_y obj_z")
# Image coordinates in pixel space
img_x, img_y = sp.symbols("img_x img_y")

intrinsic = sp.Matrix([[fx, sh, tx], [0, fy, ty], [0, 0, 1]])
obj_xyz = sp.Matrix([obj_x, obj_y, obj_z])
rot = sp.Matrix([mrp_0, mrp_1, mrp_2])

obj_xyz_rot = mrp_rotate(obj_xyz, rot)


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

values = [rx, ry] + [sp.diff(rx, p) for p in params] + [sp.diff(ry, p) for p in params]
# values = [x.simplify() for x in values]

# Use common subexpression elimination
replacements, reduced_exprs = sp.cse(values)

# Output the reusable subexpressions and final simplified derivatives
print("// Reusable subexpressions:")
for lhs, rhs in replacements:
    print(f"let {lhs} = {rhs};")

print("\n// Jacobian:")
for i, expr in enumerate(reduced_exprs[2:]):
    print(f"J[{i}] = {expr}")

print("\n// Residuals:")
for i, expr in enumerate(reduced_exprs[:2]):
    print(f"J[{i}] = {expr}")
