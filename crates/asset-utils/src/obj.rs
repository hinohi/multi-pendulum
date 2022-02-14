use std::{collections::HashMap, str::FromStr};

use crate::Vertex;

pub fn parse_obj(data: &str, color: [f32; 4]) -> Result<(Vec<Vertex>, Vec<u32>), String> {
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

#[test]
fn test_parse_obj() {
    let data = r#"# Test data
v 1 2 3
v 2 3 4
v 4 5 6
vn 0 1 2
vn 3 2 1
f 1//1 2//1 3//1
f 2//1 3//2 1//1
"#;
    let color = [0.0, 0.1, 0.2, 0.3];
    let (v, e) = parse_obj(data, color).unwrap();
    assert_eq!(
        v,
        vec![
            Vertex {
                position: [1.0, 2.0, 3.0],
                normal: [0.0, 1.0, 2.0],
                color
            },
            Vertex {
                position: [2.0, 3.0, 4.0],
                normal: [0.0, 1.0, 2.0],
                color
            },
            Vertex {
                position: [4.0, 5.0, 6.0],
                normal: [0.0, 1.0, 2.0],
                color,
            },
            Vertex {
                position: [4.0, 5.0, 6.0],
                normal: [3.0, 2.0, 1.0],
                color,
            },
        ],
    );
    assert_eq!(e, vec![0, 1, 2, 1, 3, 0]);
}
