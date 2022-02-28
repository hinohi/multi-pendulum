use std::fmt::Debug;

use cgmath::{vec3, Vector3};

pub trait Dynamics: Debug {
    fn x(&self, t: f64) -> Vector3<f64>;

    fn v(&self, t: f64) -> Vector3<f64>;

    fn a(&self, t: f64) -> Vector3<f64>;
}

#[derive(Debug, Copy, Clone)]
pub struct FixedPoint(pub Vector3<f64>);

impl Dynamics for FixedPoint {
    fn x(&self, _t: f64) -> Vector3<f64> {
        self.0
    }

    fn v(&self, _t: f64) -> Vector3<f64> {
        Vector3::new(0.0, 0.0, 0.0)
    }

    fn a(&self, _t: f64) -> Vector3<f64> {
        Vector3::new(0.0, 0.0, 0.0)
    }
}

/// Quadratic Bezier curve
#[derive(Debug, Clone)]
pub struct Bezier4 {
    p0: Vector3<f64>,
    p1: Vector3<f64>,
    p2: Vector3<f64>,
    p3: Vector3<f64>,
    ut: UniformT,
}

impl Default for Bezier4 {
    fn default() -> Self {
        Bezier4 {
            p0: vec3(0.0, 0.0, 0.0),
            p1: vec3(0.0, 0.0, 0.0),
            p2: vec3(0.0, 0.0, 0.0),
            p3: vec3(0.0, 0.0, 0.0),
            ut: UniformT::new(0.0, 1.0),
        }
    }
}

impl Bezier4 {
    pub fn from_2points(
        x0: Vector3<f64>,
        v0: Vector3<f64>,
        x1: Vector3<f64>,
        t0: f64,
        t1: f64,
    ) -> Bezier4 {
        // P'(0) = v0
        let p1 = v0 + x0 * 3.0;
        // minimize jerk
        let p2 = v0 + x0 * 2.0 + x1;
        Bezier4 {
            p0: x0,
            p1,
            p2,
            p3: x1,
            ut: UniformT::new(t0, t1),
        }
    }
}

impl Dynamics for Bezier4 {
    fn x(&self, t: f64) -> Vector3<f64> {
        let t = self.ut.t(t);
        let s = 1.0 - t;
        self.p0 * (s * s * s)
            + self.p1 * (s * s * t)
            + self.p2 * (s * t * t)
            + self.p3 * (t * t * t)
    }

    fn v(&self, t: f64) -> Vector3<f64> {
        let t = self.ut.t(t);
        let s = 1.0 - t;
        self.p0 * (-3.0 * s * s)
            + self.p1 * (s * (1.0 - 3.0 * t))
            + self.p2 * (t * (2.0 - 3.0 * t))
            + self.p3 * (3.0 * t * t)
    }

    fn a(&self, t: f64) -> Vector3<f64> {
        let t = self.ut.t(t);
        let s = 1.0 - t;
        self.p0 * (6.0 * s)
            + self.p1 * (6.0 * t - 4.0)
            + self.p2 * (2.0 - 6.0 * t)
            + self.p3 * (6.0 * t)
    }
}

#[derive(Debug, Clone)]
pub struct Oscillate1d {
    a: Vector3<f64>,
    b: Vector3<f64>,
    omega: f64,
    pub theta0: f64,
}

impl Oscillate1d {
    pub fn new(a: Vector3<f64>, b: Vector3<f64>, omega: f64, theta0: f64) -> Oscillate1d {
        Oscillate1d {
            a,
            b,
            omega,
            theta0,
        }
    }

    fn sin_cos(&self, t: f64) -> (f64, f64) {
        let theta = self.omega * (t - self.theta0);
        theta.sin_cos()
    }
}

impl Dynamics for Oscillate1d {
    fn x(&self, t: f64) -> Vector3<f64> {
        let (sin, cos) = self.sin_cos(t);
        self.a * cos + self.b * sin
    }

    fn v(&self, t: f64) -> Vector3<f64> {
        let (sin, cos) = self.sin_cos(t);
        (-self.a * sin + self.b * cos) * self.omega
    }

    fn a(&self, t: f64) -> Vector3<f64> {
        let (sin, cos) = self.sin_cos(t);
        (-self.a * cos - self.b * sin) * self.omega * self.omega
    }
}

#[derive(Debug, Copy, Clone)]
struct UniformT {
    t0: f64,
    dt: f64,
}

impl UniformT {
    #[inline]
    fn new(t0: f64, t1: f64) -> UniformT {
        UniformT { t0, dt: t1 - t0 }
    }

    #[inline]
    fn t(&self, t: f64) -> f64 {
        (t - self.t0) / self.dt
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bezier() {
        let x0 = vec3(1.0, 2.0, 3.0);
        let v0 = vec3(0.0, 0.0, 0.0);
        let x1 = vec3(-2.0, 4.0, 5.0);
        let t0 = 3.0;
        let t1 = 5.0;
        let b1 = Bezier4::from_2points(x0, v0, x1, t0, t1);
        assert_eq!(b1.x(t0), x0);
        assert_eq!(b1.x(t1), x1);
        assert_eq!(b1.v(t0), v0);

        let x2 = vec3(-1.0, 3.0, 4.0);
        let t2 = 6.0;
        let b2 = Bezier4::from_2points(b1.x(t1), b1.v(t1), x2, t1, t2);
        assert_eq!(b1.x(t1), b2.x(t1));
        assert_eq!(b1.v(t1), b2.v(t1));
        assert_eq!(b2.x(t2), x2);
    }
}
