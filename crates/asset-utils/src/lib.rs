use bytemuck::{Pod, Zeroable};

pub use crate::{obj::parse_obj, plane::make_grid};

mod obj;
mod plane;

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialOrd, PartialEq, Zeroable, Pod)]
pub struct Vertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub color: [f32; 4],
}
