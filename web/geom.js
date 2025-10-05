
export function quat_to_mat(q, out = new Float32Array(9)) {
    const w = q[0];
    const x = q[1];
    const y = q[2];
    const z = q[3];

    const norm = w * w + x * x + y * y + z * z;
    const s = norm > 0 ? 2.0 / norm : 0.0;

    const wx = w * x * s, wy = w * y * s, wz = w * z * s;
    const xx = x * x * s, xy = x * y * s, xz = x * z * s;
    const yy = y * y * s, yz = y * z * s, zz = z * z * s;

    out[0] = 1 - (yy + zz);
    out[1] = xy - wz;
    out[2] = xz + wy;

    out[3] = xy + wz;
    out[4] = 1 - (xx + zz);
    out[5] = yz - wx;

    out[6] = xz - wy;
    out[7] = yz + wx;
    out[8] = 1 - (xx + yy);

    return out;
}

export function matmul3x3(a, b, out = new Float32Array(9)) {
    out[0] = a[0] * b[0] + a[1] * b[3] + a[2] * b[6];
    out[1] = a[0] * b[1] + a[1] * b[4] + a[2] * b[7];
    out[2] = a[0] * b[2] + a[1] * b[5] + a[2] * b[8];
    out[3] = a[3] * b[0] + a[4] * b[3] + a[5] * b[6];
    out[4] = a[3] * b[1] + a[4] * b[4] + a[5] * b[7];
    out[5] = a[3] * b[2] + a[4] * b[5] + a[5] * b[8];
    out[6] = a[6] * b[0] + a[7] * b[3] + a[8] * b[6];
    out[7] = a[6] * b[1] + a[7] * b[4] + a[8] * b[7];
    out[8] = a[6] * b[2] + a[7] * b[5] + a[8] * b[8];
    return out;
}

export function mat_apply_vec(m, v, out = new Float32Array(3)) {
    out[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
    out[1] = m[3] * v[0] + m[4] * v[1] + m[5] * v[2];
    out[2] = m[6] * v[0] + m[7] * v[1] + m[8] * v[2];
    return out;
}

export function create_fast_params(intrinsic, dist_coeffs, out = new Float32Array(9)) {
    out[0] = intrinsic[0];   // fx
    out[1] = intrinsic[4];   // fy
    out[2] = intrinsic[2];   // tx
    out[3] = intrinsic[5];   // ty
    out[4] = dist_coeffs[0]; // k1;
    out[5] = dist_coeffs[1]; // k2;
    out[6] = dist_coeffs[2]; // p1;
    out[7] = dist_coeffs[3]; // p2;
    out[8] = dist_coeffs[4]; // k3;
    return out;
}

export function fast_obj_to_pix(obj_xyz, fast_params, out = new Float32Array(2)) {

    const fx = fast_params[0];
    const fy = fast_params[1];
    const tx = fast_params[2];
    const ty = fast_params[3];
    const k1 = fast_params[4];
    const k2 = fast_params[5];
    const p1 = fast_params[6];
    const p2 = fast_params[7];
    const k3 = fast_params[8];

    // Perspective projection
    const x = obj_xyz[0] / obj_xyz[2];
    const y = obj_xyz[1] / obj_xyz[2];

    // Distortion
    const r2 = x * x + y * y;
    const r4 = r2 * r2;
    const r6 = r2 * r4;
    const d = 1 + k1 * r2 + k2 * r4 + k3 * r6;
    let x_dist = x * d + (2.0 * p1 * x * y + p2 * (r2 + 2 * x * x));
    let y_dist = y * d + (2.0 * p2 * x * y + p1 * (r2 + 2 * y * y));
    x_dist = (x_dist * fx) + tx;
    y_dist = (y_dist * fy) + ty;

    out[0] = x_dist;
    out[1] = y_dist;
    return out;
}
