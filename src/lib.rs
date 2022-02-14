use asset_utils::make_grid;
use cgmath::{
    perspective, vec3, vec4, Deg, InnerSpace, Matrix, Matrix3, Matrix4, Quaternion, Rotation,
    Rotation3, SquareMatrix, Vector3,
};
use eom_sim::runge_kutta::RK4;
use itertools::Itertools;
use num_traits::One;
use pendulum::{FixedPoint, Pendulum};
use wasm_bindgen::prelude::*;
use web_sys::{console, HtmlCanvasElement};

pub use crate::user_input::Mouse;
use crate::{
    renderer::{Backend, Object},
    user_input::MouseButton,
};

mod renderer;
mod user_input;

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
    floor: Object,
    // UI
    quaternion: Quaternion<f64>,
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
            backend.make_from_obj(include_str!("assets/ico_sphere.obj"), [0.9, 0.4, 0.4, 1.0])?;
        let cylinder =
            backend.make_from_obj(include_str!("assets/cylinder.obj"), [0.1, 0.9, 0.1, 1.0])?;
        let floor = {
            let (v, e) = make_grid(
                [-25.0, -3.0, -25.0],
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 0.0],
                51,
                51,
                [0.8, 0.8, 1.0, 1.0],
                [1.0, 1.0, 0.8, 1.0],
            );
            backend.make_object(&v, &e)?
        };

        let g = vec3(0.0, 9.8, 0.0);
        let root = vec3(0.0, 0.0, 0.0);
        let length_mass = vec![(0.1, 1.0), (0.1, 2.0), (0.3, 1.0)];
        let pendulum = Pendulum::new(g, &length_mass).map_err(|s| JsValue::from_str(&s))?;
        let mut position = Vec::with_capacity(length_mass.len());
        let mut direction = vec3(1.0, 0.0, 0.0).normalize();
        for &(l, _) in length_mass.iter() {
            let last = position.last().copied().unwrap_or(root);
            position.push(last + direction * l);
            direction = Matrix3::from_angle_y(Deg(30.0)) * direction;
        }
        let velocity = vec![vec3(0.0, 0.0, 0.0); length_mass.len()];

        Ok(App {
            // GL
            backend,
            sphere,
            cylinder,
            floor,
            // UI
            quaternion: Quaternion::one(),
            // physics
            pendulum,
            root,
            position,
            velocity,
            last_tick: None,
        })
    }

    #[wasm_bindgen]
    pub fn tick(&mut self, timestamp_ms: f64, mouse: &Mouse) -> Result<(), JsValue> {
        let width = 600;
        let height = 600;
        self.backend.set_size(width, height);

        let t = timestamp_ms / 1000.0;
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

        if let Some((x, y)) = mouse.drag(MouseButton::Middle) {
            if x != 0 || y != 0 {
                let x = x as f64 / width as f64;
                let y = y as f64 / width as f64;
                let m = Matrix3::from(self.quaternion).transpose();
                let axis = m.y * x + m.x * y;
                self.quaternion = self.quaternion
                    * Quaternion::from_axis_angle(
                        axis.normalize(),
                        Deg(360.0 * (x * x + y * y).sqrt()),
                    );
            }
        }

        let projection_matrix = perspective(Deg(60.0), width as f64 / height as f64, 0.1, 10000.0);
        let view_matrix =
            Matrix4::from_translation(vec3(0.0, 0.0, -1.5)) * Matrix4::from(self.quaternion);
        self.backend.draw(
            projection_matrix * view_matrix,
            vec3(1.0, 1.0, 0.0),
            &self.calc_objects_matrix(),
        );
        Ok(())
    }

    #[wasm_bindgen]
    pub fn potential_energy(&self) -> f64 {
        self.pendulum.potential_energy(&self.position)
    }

    #[wasm_bindgen]
    pub fn kinetic_energy(&self) -> f64 {
        self.pendulum.kinetic_energy(&self.velocity)
    }

    #[wasm_bindgen]
    pub fn unit_energy(&self) -> f64 {
        self.pendulum.unit_energy()
    }
}

impl App {
    fn calc_objects_matrix(&self) -> [(&Object, Vec<Matrix4<f64>>); 3] {
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
        [
            (&self.floor, vec![Matrix4::identity()]),
            (&self.cylinder, cylinder_mat),
            (&self.sphere, sphere_mat),
        ]
    }
}
