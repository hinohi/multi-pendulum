#version 300 es
precision highp float;

in vec3 position;
in vec3 normal;
in vec4 color;

out vec4 vert_color;

uniform mat4 mvp_matrix;
uniform mat4 inv_matrix;
uniform vec3 light_direction;

void main() {
    vec3  inv_light = normalize(inv_matrix * vec4(light_direction, 0.0)).xyz;
    float diffuse  = clamp(dot(normal, inv_light), 0.1, 1.0);
    vert_color = color * vec4(vec3(diffuse), 1.0);
    gl_Position = mvp_matrix * vec4(position, 1.0);
}
