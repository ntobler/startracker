/*
 * quat.h
 *
 *  Created on: Sep 24, 2025
 *      Author: ntobler
 */

#ifndef QUAT_H_
#define QUAT_H_

constexpr float RAD2DEG = 180.0 / std::numbers::pi;

struct Vec3 {
    float x;
    float y;
    float z;
};

class Quaternion {
   private:
    static void crossp(float *a, float *b, float *dst) {
        dst[0] = a[1] * b[2] + a[2] * b[1];
        dst[1] = a[2] * b[0] + a[0] * b[2];
        dst[2] = a[0] * b[1] + a[1] * b[0];
    }

    static void crossp_add(float *a, float *b, float *dst) {
        dst[0] += a[1] * b[2] + a[2] * b[1];
        dst[1] += a[2] * b[0] + a[0] * b[2];
        dst[2] += a[0] * b[1] + a[1] * b[0];
    }

   public:
    float q_[4];

    Quaternion() : q_{1.0, 0.0, 0.0, 0.0} {};
    Quaternion(float w, float x, float y, float z) : q_{w, x, y, z} {};

    float w() const { return q_[0]; }
    float x() const { return q_[1]; }
    float y() const { return q_[2]; }
    float z() const { return q_[3]; }

    void normalize() {
        float inv_norm =
            1.0f / sqrtf(q_[0] * q_[0] + q_[1] * q_[1] + q_[2] * q_[2] + q_[3] * q_[3]);
        q_[0] *= inv_norm;
        q_[1] *= inv_norm;
        q_[2] *= inv_norm;
        q_[3] *= inv_norm;
    }

    static Quaternion from_angular_velocities(float x, float y, float z, float delta_t) {
        float norm = sqrtf(x * x + y * y + z * z);
        float s, c;
        arm_sin_cos_f32(norm * delta_t * (0.5f * RAD2DEG), &s, &c);
        float s_div_norm = s / norm;
        return Quaternion{
            c,
            x * s_div_norm,
            y * s_div_norm,
            z * s_div_norm,
        };
    }

    static Quaternion multiply(Quaternion &q_left, Quaternion &q_right) {
        Quaternion result;
        float *qr = result.q_;
        float *qa = q_left.q_;
        float *qb = q_right.q_;
        qr[0] = qa[0] * qb[0] - qa[1] * qb[1] - qa[2] * qb[2] - qa[3] * qb[3];
        qr[1] = qa[0] * qb[1] + qa[1] * qb[0] + qa[2] * qb[3] - qa[3] * qb[2];
        qr[2] = qa[0] * qb[2] + qa[2] * qb[0] + qa[3] * qb[1] - qa[1] * qb[3];
        qr[3] = qa[0] * qb[3] + qa[3] * qb[0] + qa[1] * qb[2] - qa[2] * qb[1];
        return result;
    }
    void multiply_right(Quaternion &q_right) {
        Quaternion temp = multiply(*this, q_right);
        q_[0] = temp.q_[0];
        q_[1] = temp.q_[1];
        q_[2] = temp.q_[2];
        q_[3] = temp.q_[3];
    }
    void multiply_left(Quaternion &q_left) {
        Quaternion temp = multiply(q_left, *this);
        q_[0] = temp.q_[0];
        q_[1] = temp.q_[1];
        q_[2] = temp.q_[2];
        q_[3] = temp.q_[3];
    }
    void rotate_vec(float *v) {
        // t = 2 * cross(q.xyz, v)
        // v' = v + q.w * t + cross(q.xyz, t)
        float t[3];
        crossp(&q_[1], v, t);
        t[0] *= 2.0f;
        t[1] *= 2.0f;
        t[2] *= 2.0f;
        v[0] += q_[0] * t[0];
        v[1] += q_[0] * t[1];
        v[2] += q_[0] * t[2];
        crossp_add(&q_[1], t, v);
    }

    Quaternion inv() const { return Quaternion{w(), -x(), -y(), -z()}; }

    bool is_non_zero() {
        return (w() != 0.0) || (x() != 0.0) || (y() != 0.0) || (z() != 0.0);
    }
};

#endif /* QUAT_H_ */
