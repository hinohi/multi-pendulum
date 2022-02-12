use cgmath::{perspective, vec3, vec4, Deg, InnerSpace, Matrix4, Quaternion, Rotation};
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
    backend: Backend,
    sphere: Object,
    cylinder: Object,
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
        Ok(App {
            backend,
            sphere,
            cylinder,
            last_tick: None,
        })
    }

    #[wasm_bindgen]
    pub fn tick(&mut self, timestamp: f64) -> Result<(), JsValue> {
        self.last_tick.replace(timestamp);

        let width = 800;
        let height = 600;
        self.backend.set_size(width, height);

        let projection_matrix = perspective(Deg(60.0), width as f64 / height as f64, 0.1, 10000.0);
        let view_matrix = Matrix4::from_translation(vec3(0.0, 0.0, -20.0));
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
        let position = [
            vec3(0.0, 0.0, 0.0),
            vec3(1.0, -3.0, 0.0),
            vec3(-5.0, -4.0, 2.0),
        ];
        let mut sphere_mat = Vec::new();
        let mut cylinder_mat = Vec::new();
        for i in 1..position.len() {
            let a = position[i - 1];
            let b = position[i];
            let ab = b - a;
            // cylinder's top is (0, 1, 0), bottom is (0, -1, 0).
            let rot = Matrix4::from(Quaternion::between_vectors(
                vec3(0.0, -1.0, 0.0),
                ab.normalize(),
            ));
            let scale = Matrix4::from_nonuniform_scale(0.2, ab.magnitude() / 2.0, 0.2);
            let t = rot * scale * vec4(0.0, 1.0, 0.0, 1.0);
            let t = a - vec3(t.x, t.y, t.z);
            cylinder_mat.push(Matrix4::from_translation(t) * rot * scale);
            sphere_mat.push(Matrix4::from_translation(b) * rot);
        }
        [(&self.cylinder, cylinder_mat), (&self.sphere, sphere_mat)]
    }
}
