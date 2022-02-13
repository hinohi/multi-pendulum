use cgmath::{vec3, InnerSpace, Vector3};
use eom_sim::{Eom, Explicit, ModelSpec};
use itertools::Itertools;
use std::fmt::Debug;

use crate::dynamics::{Dynamics, FixedPoint};

#[derive(Debug)]
pub struct Pendulum {
    g: Vector3<f64>,
    length: Vec<f64>,
    mass: Vec<f64>,
    unit_time: f64,
    unit_length: f64,
    root: Box<dyn Dynamics>,
}

impl Pendulum {
    pub fn new(g: Vector3<f64>, length_mass: &[(f64, f64)]) -> Result<Pendulum, String> {
        if length_mass.len() <= 1 {
            return Err(String::from("N must be grater than 1"));
        }
        let mut unit_length = f64::INFINITY;
        let mut unit_mass = 0.0;
        for (l, m) in length_mass {
            if *l <= 0.0 {
                return Err(String::from("length must be positive"));
            }
            if *m <= 0.0 {
                return Err(String::from("mass must be positive"));
            }
            unit_length = unit_length.min(*l);
            unit_mass += *m;
        }
        let length = length_mass
            .iter()
            .map(|(l, _)| *l / unit_length)
            .collect::<Vec<_>>();
        let mass = length_mass
            .iter()
            .map(|(_, m)| *m / unit_mass)
            .collect::<Vec<_>>();
        let unit_time = (unit_length / g.magnitude()).sqrt();
        Ok(Pendulum {
            g: g.normalize(),
            length,
            mass,
            unit_length,
            unit_time,
            root: Box::new(FixedPoint(vec3(0.0, 0.0, 0.0))),
        })
    }
}

impl Pendulum {
    fn calc_acceleration(
        &self,
        t: f64,
        x: &[Vector3<f64>],
        v: &[Vector3<f64>],
    ) -> Vec<Vector3<f64>> {
        let n = x.len();
        let t = t * self.unit_time;

        let mut xx = Vec::with_capacity(n);
        xx.push(x[0] - self.root.x(t));
        for (a, b) in x.iter().tuple_windows() {
            xx.push(b - a);
        }
        let mut vv = Vec::with_capacity(n);
        vv.push(v[0] - self.root.v(t));
        for (a, b) in v.iter().tuple_windows() {
            vv.push(b - a);
        }

        let lambda = {
            let n = xx.len();
            let mut a = Vec::with_capacity(n);
            a.push(xx[0].magnitude2() / self.mass[0]);
            for (xx, (&ma, &mb)) in xx.iter().zip(self.mass.iter().tuple_windows()) {
                a.push(xx.magnitude2() * (ma + mb) / (ma * mb));
            }
            let mut b = Vec::with_capacity(n - 1);
            for ((xa, xb), &m) in xx.iter().tuple_windows().zip(self.mass.iter()) {
                b.push(xa.dot(*xb) / m);
            }
            let mut c = Vec::with_capacity(n);
            c.push(vv[0].magnitude2() - xx[0].dot(self.g + self.root.a(t)));
            for vv in v.iter() {
                c.push(vv.magnitude2());
            }
            thomas(&a, &b, &c)
        };
        debug_assert_eq!(lambda.len(), n - 2);

        let mut a = Vec::with_capacity(n);
        for ((&m, (xa, xb)), (&la, &lb)) in self
            .mass
            .iter()
            .zip(xx.iter().tuple_windows())
            .zip(lambda.iter().tuple_windows())
        {
            a.push((xb * lb - xa * la) / m - self.g);
        }
        a.push(-xx[n - 2] * lambda[n - 2] / self.mass[n - 1] - self.g);
        a
    }

    pub fn tick<E: Explicit<Pendulum>>(
        &mut self,
        ticker: &mut E,
        time_start: f64,
        time_end: f64,
        root: Box<dyn Dynamics>,
        position: &mut [Vector3<f64>],
        velocity: &mut [Vector3<f64>],
    ) -> f64 {
        assert_eq!(position.len(), velocity.len());
        self.root = root;
        let n = position.len() * 3;
        let mut x = Vec::with_capacity(n);
        let mut v = Vec::with_capacity(n);
        for i in position.iter() {
            x.push(i.x / self.unit_length);
            x.push(i.y / self.unit_length);
            x.push(i.z / self.unit_length);
        }
        for i in velocity.iter() {
            v.push(i.x * (self.unit_time / self.unit_length));
            v.push(i.y * (self.unit_time / self.unit_length));
            v.push(i.z * (self.unit_time / self.unit_length));
        }

        let mut t = time_start / self.unit_time;
        let until = time_end / self.unit_time;
        let dt = (until - t) / 16.0;
        ticker.iterate_until(self, &mut t, &mut x, &mut v, dt, until);

        for (i, p) in position.iter_mut().enumerate() {
            let i = i * 3;
            p.x = x[i] * self.unit_length;
            p.y = x[i + 1] * self.unit_length;
            p.z = x[i + 2] * self.unit_length;
        }
        for (i, p) in velocity.iter_mut().enumerate() {
            let i = i * 3;
            p.x = v[i] * (self.unit_length / self.unit_time);
            p.y = v[i + 1] * (self.unit_length / self.unit_time);
            p.z = v[i + 2] * (self.unit_length / self.unit_time);
        }

        t * self.unit_time
    }
}

impl ModelSpec for Pendulum {
    type Scalar = f64;
}

impl Eom for Pendulum {
    fn acceleration(&self, t: f64, x: &[f64], v: &[f64], a: &mut [f64]) {
        fn as_vec3(v: &[f64]) -> Vec<Vector3<f64>> {
            let mut ret = Vec::with_capacity(v.len() / 3);
            for i in (0..v.len()).step_by(3) {
                ret.push(vec3(v[i], v[i + 1], v[i + 2]));
            }
            ret
        }

        let a_vec3 = self.calc_acceleration(t, &as_vec3(x), &as_vec3(v));
        for (i, v) in a_vec3.iter().enumerate() {
            a[i * 3] = v.x;
            a[i * 3 + 1] = v.y;
            a[i * 3 + 2] = v.z;
        }
    }

    fn correct(&self, _t: f64, x: &mut [f64], _v: &mut [f64]) {
        for (i, &l) in self.length.iter().enumerate() {
            let i = i * 3;
            let v = vec3(x[i], x[i + 1], x[i + 2]).normalize_to(l);
            x[i] = v.x;
            x[i + 1] = v.y;
            x[i + 2] = v.z;
        }
    }
}

/// Solving Linear Equations of a Triple Diagonal Matrix the Thomas Algorithm
///
/// ## input/output format
///
/// ex: 4-dim
///
/// ```text
/// [ a0 -b0    0   0] [x0]   [c0]
/// [-b0  a1  -b1   0] [x1] = [c1]
/// [  0 -b1   a2 -b2] [x2]   [c2]
/// [  0   0  -b2  a3] [x3]   [c3]
/// ```
#[must_use]
fn thomas(a: &[f64], b: &[f64], c: &[f64]) -> Vec<f64> {
    let n = a.len();
    let mut d = Vec::with_capacity(n + 1);
    let mut e = Vec::with_capacity(n + 1);
    d.push(b[0] / a[0]);
    e.push(c[0] / a[0]);
    for k in 1..n - 1 {
        let dk = b[k] / (a[k] - b[k - 1] * d[k - 1]);
        let ek = (c[k] + b[k - 1] * e[k - 1]) / (a[k] - b[k - 1] * d[k - 1]);
        d.push(dk);
        e.push(ek);
    }
    let mut ans = vec![0.0; n];
    ans[n - 1] = (c[n - 1] + b[n - 2] * e[n - 2]) / (a[n - 1] - b[n - 2] * d[n - 2]);
    for k in (0..n - 1).rev() {
        ans[k] = d[k] * ans[k + 1] + e[k];
    }
    ans
}

#[cfg(test)]
mod tests {
    use super::*;
    use cgmath::{assert_relative_eq, vec4, Matrix4, SquareMatrix, Vector4};

    #[test]
    fn test_thomas() {
        let a = [1.0, 2.0, 3.0, 4.0];
        let b = [3.14, 1.5, 9.2];
        let c = [2.7, 1.8, 2.81, 8.28];
        let m = Matrix4::from_cols(
            vec4(a[0], -b[0], 0.0, 0.0),
            vec4(-b[0], a[1], -b[1], 0.0),
            vec4(0.0, -b[1], a[2], -b[2]),
            vec4(0.0, 0.0, -b[2], a[3]),
        );
        let actual = thomas(&a, &b, &c);
        let actual = vec4(actual[0], actual[1], actual[2], actual[3]);
        assert_relative_eq!(
            actual,
            m.invert().unwrap() * Vector4::from(c),
            epsilon = f64::EPSILON * 4.0,
        );
    }
}
