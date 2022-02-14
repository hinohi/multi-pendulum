use cgmath::{perspective, vec3, vec4, Deg, InnerSpace, Matrix4, Quaternion, Rotation, Vector3};
use eom_sim::runge_kutta::RK4;
use itertools::Itertools;
use pendulum::{FixedPoint, Pendulum};
use wasm_bindgen::prelude::*;
use web_sys::{console, HtmlCanvasElement};

use crate::renderer::{Backend, Object};

mod renderer;

#[allow(dead_code)]
pub fn log(s: String) {
    console::log_1(&s.into());
}

#[wasm_bindgen]
pub struct App {
    // GL
    backend: Backend,
    sphere: Object,
    cylinder: Object,
    // physics
    pendulum: Pendulum,
    root: Vector3<f64>,
    position: Vec<Vector3<f64>>,
    velocity: Vec<Vector3<f64>>,
    last_tick: Option<f64>,
}

#[wasm_bindgen]
impl App {
    #[wasm_bindgen(constructor)]
    pub fn new(canvas: HtmlCanvasElement) -> Result<App, JsValue> {
        let backend = Backend::new(canvas)?;
        let sphere =
            backend.make_object(include_str!("assets/ico_sphere.obj"), [0.9, 0.4, 0.4, 1.0])?;
        let cylinder =
            backend.make_object(include_str!("assets/cylinder.obj"), [0.1, 0.9, 0.1, 1.0])?;

        let root = vec3(0.0, 0.0, 0.0);
        let length_mass = vec![(0.1, 1.0), (0.2, 2.0)];
        let pendulum =
            Pendulum::new(vec3(0.0, 9.8, 0.0), &length_mass).map_err(|s| JsValue::from_str(&s))?;
        let mut position = Vec::with_capacity(length_mass.len());
        let direction = vec3(3.0, -1.0, 0.0).normalize();
        for &(l, _) in length_mass.iter() {
            let last = position.last().copied().unwrap_or(root);
            position.push(last + direction * l);
            // direction = Matrix3::from_angle_y(Deg(30.0)) * direction;
        }
        let velocity = vec![vec3(0.0, 0.0, 0.0); length_mass.len()];

        Ok(App {
            backend,
            sphere,
            cylinder,
            pendulum,
            root,
            position,
            velocity,
            last_tick: None,
        })
    }

    #[wasm_bindgen]
    pub fn tick(&mut self, timestamp_ms: f64) -> Result<(), JsValue> {
        let t = timestamp_ms / 1000.0 / 10.0;
        let last_tick = self.last_tick.replace(t).unwrap_or(t);

        let new_tick = self.pendulum.tick(
            &mut RK4::new(),
            last_tick,
            t,
            Box::new(FixedPoint(self.root)),
            &mut self.position,
            &mut self.velocity,
        );
        self.last_tick.replace(new_tick);

        let width = 800;
        let height = 600;
        self.backend.set_size(width, height);

        let projection_matrix = perspective(Deg(60.0), width as f64 / height as f64, 0.1, 10000.0);
        let view_matrix = Matrix4::from_translation(vec3(0.0, 0.0, -10.0));
        self.backend.draw(
            projection_matrix * view_matrix,
            vec3(1.0, 1.0, 0.0),
            &self.calc_objects_matrix(),
        );
        Ok(())
    }
}

impl App {
    fn calc_objects_matrix(&self) -> [(&Object, Vec<Matrix4<f64>>); 2] {
        let mut position = vec![self.root];
        position.extend_from_slice(&self.position);

        let global_scale = 0.05;

        let mut sphere_mat = Vec::new();
        let mut cylinder_mat = Vec::new();
        for (&a, &b) in position.iter().tuple_windows() {
            let ab = b - a;
            // cylinder's top is (0, 1, 0), bottom is (0, -1, 0).
            let rot = Matrix4::from(Quaternion::between_vectors(
                vec3(0.0, -1.0, 0.0),
                ab.normalize(),
            ));
            let scale = Matrix4::from_nonuniform_scale(
                0.05 * global_scale,
                ab.magnitude() / 2.0,
                0.05 * global_scale,
            );
            let t = rot * scale * vec4(0.0, 1.0, 0.0, 1.0);
            let t = a - vec3(t.x, t.y, t.z);
            cylinder_mat.push(Matrix4::from_translation(t) * rot * scale);

            let scale = Matrix4::from_scale(0.4 * global_scale);
            sphere_mat.push(Matrix4::from_translation(b) * rot * scale);
        }
        [(&self.cylinder, cylinder_mat), (&self.sphere, sphere_mat)]
    }
}
