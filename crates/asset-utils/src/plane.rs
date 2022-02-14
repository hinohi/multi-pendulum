use std::collections::HashMap;

use crate::Vertex;

pub fn make_grid(
    o: [f32; 3],
    axis0: [f32; 3],
    axis1: [f32; 3],
    n0: u32,
    n1: u32,
    color0: [f32; 4],
    color1: [f32; 4],
) -> (Vec<Vertex>, Vec<u32>) {
    let mut mem = HashMap::new();
    let mut vertex_array = Vec::new();
    let mut element_array = Vec::new();
    let mut color_i = 0;
    let color = [color0, color1];
    let normal = [
        axis0[1] * axis1[2] - axis0[2] * axis1[1],
        axis0[2] * axis1[0] - axis0[0] * axis1[2],
        axis0[0] * axis1[1] - axis0[1] * axis1[0],
    ];
    for i0 in 0..n0 {
        for i1 in 0..n1 {
            for (k0, k1) in [(0, 0), (1, 0), (0, 1), (1, 0), (1, 1), (0, 1)] {
                let i0 = i0 + k0;
                let i1 = i1 + k1;
                let index = *mem.entry((i0, i1, color_i)).or_insert_with(|| {
                    let position = [
                        o[0] + axis0[0] * i0 as f32 + axis1[0] * i1 as f32,
                        o[1] + axis0[1] * i0 as f32 + axis1[1] * i1 as f32,
                        o[2] + axis0[2] * i0 as f32 + axis1[2] * i1 as f32,
                    ];
                    vertex_array.push(Vertex {
                        position,
                        normal,
                        color: color[color_i],
                    });
                    (vertex_array.len() - 1) as u32
                });
                element_array.push(index);
            }
            color_i = 1 - color_i;
        }
    }
    (vertex_array, element_array)
}
