use std::collections::HashMap;
use std::rc::Rc;
use std::str::FromStr;

use bytemuck::{Pod, Zeroable};
use cgmath::{Matrix4, SquareMatrix, Vector3};
use glow::{Context, HasContext, WebBufferKey, WebProgramKey, WebShaderKey};
use memoffset::offset_of;
use wasm_bindgen::{prelude::*, JsCast};
use web_sys::{HtmlCanvasElement, WebGl2RenderingContext, WebGlUniformLocation};

use crate::log;

#[derive(Debug)]
pub struct Backend {
    gl: Rc<Context>,
    canvas: HtmlCanvasElement,
}

#[derive(Debug)]
pub struct Object {
    gl: Rc<Context>,
    program: WebProgramKey,
    mvp_matrix_location: WebGlUniformLocation,
    inv_matrix_location: WebGlUniformLocation,
    light_direction_location: WebGlUniformLocation,
    vbo: WebBufferKey,
    ebo: WebBufferKey,
    elements: usize,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Zeroable, Pod)]
pub struct Vertex {
    position: [f32; 3],
    normal: [f32; 3],
    color: [f32; 4],
}

impl Backend {
    pub fn new(canvas: HtmlCanvasElement) -> Result<Backend, JsValue> {
        let webgl = canvas
            .get_context("webgl2")?
            .ok_or("webgl2 not supported")?
            .dyn_into::<WebGl2RenderingContext>()?;
        let gl = Rc::new(Context::from_webgl2_context(webgl));

        unsafe {
            gl.enable(glow::DEPTH_TEST);
            gl.clear_color(0.9, 0.9, 0.9, 1.0);
            gl.clear(glow::COLOR_BUFFER_BIT | glow::DEPTH_BUFFER_BIT);
        }

        Ok(Backend { gl, canvas })
    }

    pub fn set_size(&self, width: u32, height: u32) {
        self.canvas.set_width(width);
        self.canvas.set_height(height);
        unsafe {
            self.gl.viewport(0, 0, width as i32, height as i32);
        }
    }

    pub fn make_object(&self, data: &str, color: [f32; 4]) -> Result<Object, JsValue> {
        Object::new(self.gl.clone(), data, color).map_err(|err| JsValue::from_str(&err))
    }

    pub fn draw(
        &self,
        view_projection_matrix: Matrix4<f64>,
        light_direction: Vector3<f64>,
        objects: &[(&Object, Matrix4<f64>)],
    ) {
        unsafe {
            self.gl
                .clear(glow::COLOR_BUFFER_BIT | glow::DEPTH_BUFFER_BIT);
        }
        for &(obj, mat) in objects {
            obj.draw(view_projection_matrix, mat, light_direction);
        }
        unsafe {
            self.gl.flush();
        }
    }
}

impl Object {
    pub fn new(gl: Rc<Context>, data: &str, color: [f32; 4]) -> Result<Object, String> {
        let program = make_program(
            &gl,
            include_str!("assets/vertex_shader.glsl"),
            include_str!("assets/fragment_shader.glsl"),
        )?;
        let (vertex_array, element_array) = parse_obj(data, color)?;
        let vbo = make_buffer(&gl, glow::ARRAY_BUFFER, bytemuck::cast_slice(&vertex_array))?;
        let ebo = make_buffer(
            &gl,
            glow::ELEMENT_ARRAY_BUFFER,
            bytemuck::cast_slice(&element_array),
        )?;

        make_vertex_attrib(
            &gl,
            program,
            "position",
            3,
            std::mem::size_of::<Vertex>(),
            offset_of!(Vertex, position),
        )?;
        make_vertex_attrib(
            &gl,
            program,
            "normal",
            3,
            std::mem::size_of::<Vertex>(),
            offset_of!(Vertex, normal),
        )?;
        make_vertex_attrib(
            &gl,
            program,
            "color",
            4,
            std::mem::size_of::<Vertex>(),
            offset_of!(Vertex, color),
        )?;

        let mvp_matrix_location = get_uniform_location(&gl, program, "mvp_matrix")?;
        let inv_matrix_location = get_uniform_location(&gl, program, "inv_matrix")?;
        let light_direction_location = get_uniform_location(&gl, program, "light_direction")?;

        Ok(Object {
            gl,
            program,
            mvp_matrix_location,
            inv_matrix_location,
            light_direction_location,
            vbo,
            ebo,
            elements: element_array.len(),
        })
    }

    pub fn draw(
        &self,
        view_projection_matrix: Matrix4<f64>,
        model_matrix: Matrix4<f64>,
        light_direction: Vector3<f64>,
    ) {
        fn mat_as_vec(mat: Matrix4<f64>) -> Vec<f32> {
            let mut v = Vec::with_capacity(16);
            for col in Into::<[[f64; 4]; 4]>::into(mat) {
                for c in col {
                    v.push(c as f32);
                }
            }
            v
        }

        let inv_matrix = model_matrix.invert().unwrap_or(Matrix4::identity());
        let mvp_matrix = view_projection_matrix * inv_matrix;
        unsafe {
            self.gl.use_program(Some(self.program));
            self.gl.uniform_matrix_4_f32_slice(
                Some(&self.mvp_matrix_location),
                false,
                &mat_as_vec(mvp_matrix),
            );
            self.gl.uniform_matrix_4_f32_slice(
                Some(&self.inv_matrix_location),
                false,
                &mat_as_vec(inv_matrix),
            );
            self.gl.uniform_3_f32(
                Some(&self.light_direction_location),
                light_direction.x as f32,
                light_direction.y as f32,
                light_direction.z as f32,
            );
            self.gl
                .draw_elements(glow::TRIANGLES, self.elements as i32, glow::UNSIGNED_INT, 0);
        }
    }
}

fn make_shader(
    gl: &Context,
    program: WebProgramKey,
    sharder_type: u32,
    source: &str,
) -> Result<WebShaderKey, String> {
    unsafe {
        let sharder = gl.create_shader(sharder_type)?;
        gl.shader_source(sharder, source);
        gl.compile_shader(sharder);
        if !gl.get_shader_compile_status(sharder) {
            return Err(gl.get_shader_info_log(sharder));
        }
        gl.attach_shader(program, sharder);
        Ok(sharder)
    }
}

fn make_program(
    gl: &Context,
    vertex_shader_source: &str,
    fragment_shader_source: &str,
) -> Result<WebProgramKey, String> {
    unsafe {
        let program = gl.create_program()?;
        let vertex_shader = make_shader(gl, program, glow::VERTEX_SHADER, vertex_shader_source)?;
        let fragment_shader =
            make_shader(gl, program, glow::FRAGMENT_SHADER, fragment_shader_source)?;
        gl.link_program(program);
        if !gl.get_program_link_status(program) {
            return Err(gl.get_program_info_log(program));
        }
        gl.detach_shader(program, vertex_shader);
        gl.delete_shader(vertex_shader);
        gl.detach_shader(program, fragment_shader);
        gl.delete_shader(fragment_shader);
        gl.use_program(Some(program));
        Ok(program)
    }
}

fn make_vertex_attrib(
    gl: &Context,
    program: WebProgramKey,
    name: &str,
    size: usize,
    stride: usize,
    offset: usize,
) -> Result<(), String> {
    unsafe {
        let index = gl
            .get_attrib_location(program, name)
            .ok_or_else(|| format!("No '{}' attribute", name))?;
        gl.enable_vertex_attrib_array(index);
        gl.vertex_attrib_pointer_f32(
            index,
            size as i32,
            glow::FLOAT,
            false,
            stride as i32,
            offset as i32,
        );
        Ok(())
    }
}

fn get_uniform_location(
    gl: &Context,
    program: WebProgramKey,
    name: &str,
) -> Result<WebGlUniformLocation, String> {
    unsafe {
        gl.get_uniform_location(program, name)
            .ok_or_else(|| format!("No '{}' uniform attribute", name))
    }
}

fn make_buffer(gl: &Context, target: u32, data: &[u8]) -> Result<WebBufferKey, String> {
    unsafe {
        let bo = gl.create_buffer()?;
        gl.bind_buffer(target, Some(bo));
        gl.buffer_data_u8_slice(target, data, glow::STATIC_READ);
        Ok(bo)
    }
}

fn parse_obj(data: &str, color: [f32; 4]) -> Result<(Vec<Vertex>, Vec<u32>), String> {
    fn parse<'a, I: Iterator<Item = &'a str>, T: FromStr>(
        i: usize,
        t: &str,
        mut it: I,
    ) -> Result<T, String>
    where
        <T as FromStr>::Err: std::error::Error,
    {
        it.next()
            .ok_or_else(|| format!("line {}, type '{}': no value", i, t))?
            .parse()
            .map_err(|err| format!("line {}, type '{}': {}", i, t, err))
    }

    let mut position_array = Vec::new();
    let mut normal_array = Vec::new();
    let mut vertex_array = Vec::new();
    let mut element_array = Vec::new();
    let mut face_map = HashMap::new();
    for (i, line) in data.lines().enumerate() {
        if line.starts_with('#') {
            continue;
        }
        let mut words = line.split_ascii_whitespace();
        match words.next() {
            None => (),
            Some("v") => position_array.push([
                parse(i, "v", &mut words)?,
                parse(i, "v", &mut words)?,
                parse(i, "v", &mut words)?,
            ]),
            Some("vn") => normal_array.push([
                parse(i, "vn", &mut words)?,
                parse(i, "vn", &mut words)?,
                parse(i, "vn", &mut words)?,
            ]),
            Some("f") => {
                let words = words.collect::<Vec<_>>();
                if words.len() != 3 {
                    return Err(format!("line {}: only support triangle", i));
                }
                for word in words {
                    let mut words = word.split('/');
                    let pos_id: usize = parse(i, "face v", &mut words)?;
                    words.next(); // skip texture
                    let normal_id: usize = parse(i, "face n", &mut words)?;
                    let vert_id = *face_map.entry((pos_id, normal_id)).or_insert_with(|| {
                        vertex_array.push(Vertex {
                            position: position_array[pos_id - 1],
                            normal: normal_array[normal_id - 1],
                            color,
                        });
                        (vertex_array.len() - 1) as u32
                    });
                    element_array.push(vert_id);
                }
            }
            Some(t) => return Err(format!("line {}: unsupported type {}", i, t)),
        }
    }
    Ok((vertex_array, element_array))
}
