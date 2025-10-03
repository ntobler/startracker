use std::array;

#[derive(Clone, Copy)]
struct Value<const N: usize> {
    v: f64,
    g: [f64; N],
}

impl<const N: usize> Value<N> {
    fn constant(c: f64) -> Value<N> {
        Value { v: c, g: [0.0; N] }
    }

    fn add(&self, q: &Value<N>) -> Value<N> {
        let mut g = [0.0; N];
        for i in 0..N {
            g[i] = self.g[i] + q.g[i];
        }
        Value { v: self.v + q.v, g }
    }

    fn sub(&self, q: &Value<N>) -> Value<N> {
        let mut g = [0.0; N];
        for i in 0..N {
            g[i] = self.g[i] - q.g[i];
        }
        Value { v: self.v - q.v, g }
    }

    fn mul(&self, q: &Value<N>) -> Value<N> {
        let mut g = [0.0; N];
        for i in 0..N {
            g[i] = self.g[i] * q.v + self.v * q.g[i];
        }
        Value { v: self.v * q.v, g }
    }

    fn scale(&self, k: f64) -> Value<N> {
        Value {
            v: self.v * k,
            g: array::from_fn(|i| self.g[i] * k),
        }
    }

    fn sqrt(&self) -> Value<N> {
        let sqrtv = self.v.sqrt();
        Value {
            v: sqrtv,
            g: array::from_fn(|i| 0.5 / sqrtv * self.g[i]),
        }
    }

    fn inv(&self) -> Value<N> {
        let invv = 1.0 / self.v;
        Value {
            v: invv,
            g: array::from_fn(|i| -invv * invv * self.g[i]),
        }
    }

    fn sin(&self) -> Value<N> {
        let sv = self.v.sin();
        let cv = self.v.cos();
        Value {
            v: sv,
            g: array::from_fn(|i| cv * self.g[i]),
        }
    }

    fn cos(&self) -> Value<N> {
        let sv = self.v.sin();
        Value {
            v: self.v.cos(),
            g: array::from_fn(|i| -sv * self.g[i]),
        }
    }
}

struct Quat<const N: usize> {
    w: Value<N>,
    x: Value<N>,
    y: Value<N>,
    z: Value<N>,
}

impl<const N: usize> Quat<N> {
    fn from_constant(q: &[f64; 4]) -> Quat<N> {
        Quat::<N> {
            w: Value {
                v: q[0],
                g: [0.0; N],
            },
            x: Value {
                v: q[1],
                g: [0.0; N],
            },
            y: Value {
                v: q[2],
                g: [0.0; N],
            },
            z: Value {
                v: q[3],
                g: [0.0; N],
            },
        }
    }

    fn invert(&self) -> Quat<N> {
        Quat::<N> {
            w: self.w,
            x: Value::constant(0.0).sub(&self.x),
            y: Value::constant(0.0).sub(&self.y),
            z: Value::constant(0.0).sub(&self.z),
        }
    }

    fn normalize(&self) -> Quat<N> {
        let norm2 = self
            .w
            .mul(&self.w)
            .add(&self.x.mul(&self.x))
            .add(&self.y.mul(&self.y))
            .add(&self.z.mul(&self.z));
        let s = norm2.sqrt().inv();
        Quat::<N> {
            w: self.w.mul(&s),
            x: self.x.mul(&s),
            y: self.y.mul(&s),
            z: self.z.mul(&s),
        }
    }
}

fn quat_mul<const N: usize>(a: &Quat<N>, b: &Quat<N>) -> Quat<N> {
    Quat {
        w: a.w
            .mul(&b.w)
            .sub(&a.x.mul(&b.x))
            .sub(&a.y.mul(&b.y))
            .sub(&a.z.mul(&b.z)),
        x: a.w
            .mul(&b.x)
            .add(&a.x.mul(&b.w))
            .add(&a.y.mul(&b.z))
            .sub(&a.z.mul(&b.y)),
        y: a.w
            .mul(&b.y)
            .add(&a.y.mul(&b.w))
            .add(&a.z.mul(&b.x))
            .sub(&a.x.mul(&b.z)),
        z: a.w
            .mul(&b.z)
            .add(&a.z.mul(&b.w))
            .add(&a.x.mul(&b.y))
            .sub(&a.y.mul(&b.x)),
    }
}

fn from_angular_velocities(
    gyro: &[f64; 3],
    scale: &[f64; 3],
    bias: &[f64; 3],
    delta_t: f64,
) -> Quat<6> {
    let x = Value::<6> {
        v: (1.0 + scale[0]) * gyro[0] + bias[0],
        g: [gyro[0], 0.0, 0.0, 1.0, 0.0, 0.0],
    };
    let y = Value::<6> {
        v: (1.0 + scale[1]) * gyro[1] + bias[1],
        g: [0.0, gyro[1], 0.0, 0.0, 1.0, 0.0],
    };
    let z = Value::<6> {
        v: (1.0 + scale[2]) * gyro[2] + bias[2],
        g: [0.0, 0.0, gyro[2], 0.0, 0.0, 1.0],
    };

    let norm2 = x.mul(&x).add(&y.mul(&y)).add(&z.mul(&z));
    let norm = norm2.sqrt();
    let angle = norm.scale(0.5 * delta_t);
    let c = angle.cos();
    let s = angle.sin();
    let s_div_norm = s.mul(&norm.inv());

    Quat::<6> {
        w: c,
        x: x.mul(&s_div_norm),
        y: y.mul(&s_div_norm),
        z: z.mul(&s_div_norm),
    }
}

// #[no_mangle]
fn propagate(
    q_0: Quat<6>,
    omega_t: &[[f64; 3]],
    scale: &[f64; 3],
    bias: &[f64; 3],
    delta_t: f64,
) -> Quat<6> {
    // println!("new propagate");

    let mut q = q_0;
    for omega in omega_t {
        let dq = from_angular_velocities(omega, scale, bias, delta_t);
        q = quat_mul(&q, &dq);
        q = q.normalize();
    }
    q
}

pub fn objective_function(
    q_0: &[f64; 4],
    q_1: &[f64; 4],
    omega_t: &[[f64; 3]],
    scale: [f64; 3],
    bias: [f64; 3],
    delta_t: f64,
) -> (
    nalgebra::SVector<f64, 3>,
    nalgebra::SMatrix<f64, 3, 6>,
    [f64; 4],
) {
    let q_0 = Quat::from_constant(q_0);
    let q_1 = Quat::from_constant(q_1);
    let q_x = propagate(q_0, omega_t, &scale, &bias, delta_t);

    let res = quat_mul(&q_x, &q_1.invert());

    let q_x = [q_x.w.v, q_x.x.v, q_x.y.v, q_x.z.v];

    let residuals = nalgebra::SVector::<f64, 3>::new(res.x.v, res.y.v, res.z.v);
    let jacobian = nalgebra::SMatrix::<f64, 3, 6>::new(
        res.x.g[0], res.x.g[1], res.x.g[2], res.x.g[3], res.x.g[4], res.x.g[5], res.y.g[0],
        res.y.g[1], res.y.g[2], res.y.g[3], res.y.g[4], res.y.g[5], res.z.g[0], res.z.g[1],
        res.z.g[2], res.z.g[3], res.z.g[4], res.z.g[5],
    );

    (residuals, jacobian, q_x)
}

pub fn filter_step(
    q_0: &[f64; 4],
    q_1: &[f64; 4],
    omega_t: &[[f64; 3]],
    scale: [f64; 3],
    bias: [f64; 3],
    delta_t: f64,
    lamb: f64,
    alpha: f64,
    trust_region: f64,
) -> ([f64; 3], [f64; 3]) {
    let (residuals, jacobian, _) = objective_function(q_0, q_1, omega_t, scale, bias, delta_t);

    let jacobian_transposed = jacobian.transpose();

    let jtj = jacobian_transposed * jacobian;
    let jtr = jacobian_transposed * residuals;
    let n_rows_rows = jtj.nrows();
    let a = jtj + nalgebra::DMatrix::identity(n_rows_rows, n_rows_rows) * lamb;
    let lu = nalgebra::LU::new(a);
    let delta = lu.solve(&(-&jtr)).expect("Matrix may be singular");

    let scale_est =
        array::from_fn(|i| scale[i] + (delta[i] * alpha).clamp(-trust_region, trust_region));
    let bias_est =
        array::from_fn(|i| bias[i] + (delta[i + 3] * alpha).clamp(-trust_region, trust_region));

    (scale_est, bias_est)
}
