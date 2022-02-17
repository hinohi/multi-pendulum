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
    t0: f64,
    dt: f64,
}

impl Default for Bezier4 {
    fn default() -> Self {
        Bezier4 {
            p0: vec3(0.0, 0.0, 0.0),
            p1: vec3(0.0, 0.0, 0.0),
            p2: vec3(0.0, 0.0, 0.0),
            p3: vec3(0.0, 0.0, 0.0),
            t0: 0.0,
            dt: 1.0,
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
        let p1 = v0 / 3.0 + x0;
        // minimize jerk
        let p2 = v0 + x0 * 2.0 + x1;
        Bezier4 {
            p0: x0,
            p1,
            p2,
            p3: x1,
            t0,
            dt: t1 - t0,
        }
    }

    fn t(&self, t: f64) -> f64 {
        (t - self.t0) / self.dt
    }
}

impl Dynamics for Bezier4 {
    fn x(&self, t: f64) -> Vector3<f64> {
        let t = self.t(t);
        let s = 1.0 - t;
        self.p0 * (s * s * s)
            + self.p1 * (3.0 * s * s * t)
            + self.p2 * (3.0 * s * t * t)
            + self.p3 * (t * t * t)
    }

    fn v(&self, t: f64) -> Vector3<f64> {
        let t = self.t(t);
        let s = 1.0 - t;
        self.p0 * (-3.0 * s * s)
            + self.p1 * (-3.0 * s * (3.0 * t - 1.0))
            + self.p2 * (t * (2.0 - 3.0 * t))
            + self.p3 * (3.0 * t * t)
    }

    fn a(&self, t: f64) -> Vector3<f64> {
        let t = self.t(t);
        self.p0 * (6.0 - 6.0 * t)
            + self.p1 * (18.0 * t - 12.0)
            + self.p2 * (2.0 - 6.0 * t)
            + self.p3 * (6.0 * t)
    }
}
