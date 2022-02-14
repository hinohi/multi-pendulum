use bytemuck::{Pod, Zeroable};

pub use crate::obj::parse_obj;

mod obj;

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialOrd, PartialEq, Zeroable, Pod)]
pub struct Vertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub color: [f32; 4],
}
