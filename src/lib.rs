use asset_utils::make_grid;
use cgmath::{
    perspective, vec3, vec4, Deg, InnerSpace, Matrix, Matrix3, Matrix4, MetricSpace, Quaternion,
    Rotation, Rotation3, SquareMatrix, Vector3,
};
use eom_sim::runge_kutta::RK4;
use itertools::Itertools;
use num_traits::{One, Zero};
use pendulum::Pendulum;
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
    grab: bool,
    // physics
    pendulum: Pendulum,
    root_position: Vector3<f64>,
    root_velocity: Vector3<f64>,
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
        let root = Vector3::zero();
        let length_mass = vec![(0.3, 1.0); 4];
        let pendulum = Pendulum::new(g, &length_mass).map_err(|s| JsValue::from_str(&s))?;
        let mut position = Vec::with_capacity(length_mass.len());
        let mut direction = vec3(0.0, -1.0, 0.0).normalize();
        for &(l, _) in length_mass.iter() {
            let last = position.last().copied().unwrap_or(root);
            position.push(last + direction * l);
            direction = Matrix3::from_angle_y(Deg(30.0)) * direction;
        }

        Ok(App {
            // GL
            backend,
            sphere,
            cylinder,
            floor,
            // UI
            quaternion: Quaternion::one(),
            grab: false,
            // physics
            pendulum,
            root_position: root,
            root_velocity: Vector3::zero(),
            position,
            velocity: vec![Vector3::zero(); length_mass.len()],
            last_tick: None,
        })
    }

    #[wasm_bindgen]
    pub fn tick(&mut self, timestamp_ms: f64, mouse: &Mouse) -> Result<(), JsValue> {
        let width = 1000;
        let height = 1000;
        self.backend.set_size(width, height);

        let t = timestamp_ms / 1000.0;
        let last_tick = self.last_tick.replace(t).unwrap_or(t);
        if t == last_tick {
            return Ok(());
        }

        let m = Matrix3::from(self.quaternion).transpose();

        if let Some((x, y)) = mouse.drag(MouseButton::Middle) {
            if x != 0 || y != 0 {
                let x = x as f64 / width as f64;
                let y = y as f64 / width as f64;
                let axis = m.y * x + m.x * y;
                self.quaternion = self.quaternion
                    * Quaternion::from_axis_angle(
                        axis.normalize(),
                        Deg(360.0 * (x * x + y * y).sqrt()),
                    );
            }
        }

        let projection_matrix = perspective(Deg(90.0), width as f64 / height as f64, 0.1, 10000.0);
        let view_matrix =
            Matrix4::from_translation(vec3(0.0, 0.0, -1.5)) * Matrix4::from(self.quaternion);
        let view_projection_matrix = projection_matrix * view_matrix;

        let disp2model = |(x, y)| -> Vector3<f64> {
            let (x, y) = (
                x as f64 / width as f64 * 2.0 - 1.0,
                1.0 - y as f64 / height as f64 * 2.0,
            );
            let root_in_display = view_projection_matrix
                * vec4(
                    self.root_position.x,
                    self.root_position.y,
                    self.root_position.z,
                    1.0,
                );
            let v = view_projection_matrix.invert().unwrap()
                * vec4(x, y, root_in_display.z, root_in_display.w);
            // I don't know why this √2 factor needed.
            vec3(v.x, v.y, v.z) * std::f64::consts::SQRT_2
        };

        let root_end = if let Some(p) = mouse.click(MouseButton::Left) {
            let click_in_model = disp2model(p);
            if self.root_position.distance2(click_in_model) <= 1e-3 {
                self.grab = true;
            }
            if self.grab {
                disp2model(mouse.current_position().unwrap_or(p))
            } else {
                self.root_position
            }
        } else {
            self.grab = false;
            self.root_position
        };

        let (new_tick, new_root_position, new_root_velocity) = self.pendulum.tick(
            &mut RK4::new(),
            last_tick,
            t,
            self.root_position,
            self.root_velocity,
            root_end,
            &mut self.position,
            &mut self.velocity,
        );
        self.last_tick = Some(new_tick);
        self.root_position = new_root_position;
        self.root_velocity = new_root_velocity;

        self.backend.draw(
            view_projection_matrix,
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
        let mut position = vec![self.root_position];
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
