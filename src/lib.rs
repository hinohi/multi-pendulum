use cgmath::{perspective, vec3, Deg, Matrix4, SquareMatrix};
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
    // cylinder: Object,
    last_tick: Option<f64>,
}

#[wasm_bindgen]
impl App {
    #[wasm_bindgen(constructor)]
    pub fn new(canvas: HtmlCanvasElement) -> Result<App, JsValue> {
        let backend = Backend::new(canvas)?;
        let sphere =
            backend.make_object(include_str!("assets/ico_sphere.obj"), [0.9, 0.9, 0.9, 1.0])?;
        // let cylinder =
        //     backend.make_object(include_str!("assets/cylinder.obj"), [0.5, 0.5, 0.5, 1.0])?;
        Ok(App {
            backend,
            sphere,
            // cylinder,
            last_tick: None,
        })
    }

    #[wasm_bindgen]
    pub fn tick(&mut self, timestamp: f64) -> Result<(), JsValue> {
        self.last_tick.replace(timestamp);

        let width = 600;
        let height = 400;
        self.backend.set_size(width, height);

        let projection_matrix = perspective(Deg(60.0), width as f64 / height as f64, 0.1, 10000.0);
        self.backend.draw(
            projection_matrix,
            vec3(0.0, -1.0, 0.0),
            &[
                (
                    &self.sphere,
                    Matrix4::from_translation(vec3(0.0, 0.0, 10.0)),
                ),
                // (&self.cylinder, Matrix4::identity()),
            ],
        );
        Ok(())
    }
}
