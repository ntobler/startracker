"""Script to compute the Jacobian of the star tracker model with respect to its parameters.

Used to generate the optimized code in the starcal.rs file.
"""

import sympy as sp

# Define all symbols
# Intrinsic camera parameters
fx, sh, tx, fy, ty = sp.symbols("fx sh tx fy ty")
k1, k2, p1, p2, k3 = sp.symbols("k1 k2 p1 p2 k3")

# Object coordinates in the camera frame
obj_x, obj_y, obj_z = sp.symbols("obj_x obj_y obj_z")
# Image coordinates in pixel space
img_x, img_y = sp.symbols("img_x img_y")

intrinsic = sp.Matrix([[fx, sh, tx], [0, fy, ty], [0, 0, 1]])

xyz = sp.Matrix([obj_x, obj_y, obj_z])

temp = intrinsic * xyz
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

# Parameter list
params = [fx, sh, tx, fy, ty, k1, k2, p1, p2, k3]

# Jacobian
jacobian = [rx, ry] + [sp.diff(rx, p) for p in params] + [sp.diff(ry, p) for p in params]
jacobian = [x.simplify() for x in jacobian]

# Use common subexpression elimination
replacements, reduced_exprs = sp.cse(jacobian)

# Output the reusable subexpressions and final simplified derivatives
print("// Reusable subexpressions:")
for lhs, rhs in replacements:
    print(f"let {lhs} = {rhs};")

print("\n// Reduced expressions:")
for i, expr in enumerate(reduced_exprs):
    print(f"J[{i}] = {expr}")
