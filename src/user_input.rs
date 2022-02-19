use wasm_bindgen::prelude::*;

#[wasm_bindgen]
#[derive(Default, Debug)]
pub struct Mouse {
    button_downed: [Option<(i32, i32)>; 3],
    last_position: Option<(i32, i32)>,
    current_position: Option<(i32, i32)>,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum MouseButton {
    Left = 0,
    Middle = 1,
    Right = 2,
}

#[wasm_bindgen]
impl Mouse {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Mouse {
        Default::default()
    }

    #[wasm_bindgen]
    pub fn add_mousedown(&mut self, x: i32, y: i32, button: i32) {
        self.add_position(x, y);
        if 0 <= button && button <= 2 {
            self.button_downed[button as usize] = Some((x, y));
        }
    }

    #[wasm_bindgen]
    pub fn add_mouseup(&mut self, x: i32, y: i32, button: i32) {
        self.add_position(x, y);
        if 0 <= button && button <= 2 {
            self.button_downed[button as usize] = None;
        }
    }

    #[wasm_bindgen]
    pub fn add_mousemove(&mut self, x: i32, y: i32) {
        self.add_position(x, y);
    }
}

impl Mouse {
    fn add_position(&mut self, x: i32, y: i32) {
        if let Some(c) = self.current_position {
            self.last_position.replace(c);
        }
        self.current_position.replace((x, y));
    }

    pub fn current_position(&self) -> Option<(i32, i32)> {
        self.current_position
    }

    pub fn click(&self, button: MouseButton) -> Option<(i32, i32)> {
        self.button_downed[button as usize]
    }

    pub fn drag(&self, button: MouseButton) -> Option<(i32, i32)> {
        if self.button_downed[button as usize].is_none() {
            return None;
        }
        let (x0, y0) = self.last_position?;
        let (x1, y1) = self.current_position?;
        Some((x1 - x0, y1 - y0))
    }
}
