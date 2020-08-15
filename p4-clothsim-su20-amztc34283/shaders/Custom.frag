#version 330

// (Every uniform is available here.)

uniform mat4 u_view_projection;
uniform mat4 u_model;

uniform float u_normal_scaling;
uniform float u_height_scaling;

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

// Feel free to add your own textures. If you need more than 4,
// you will need to modify the skeleton.
uniform sampler2D u_texture_1;
uniform sampler2D u_texture_2;
uniform sampler2D u_texture_3;
uniform sampler2D u_texture_4;

uniform vec2 u_texture_1_size;
uniform vec2 u_texture_2_size;
uniform vec2 u_texture_3_size;
uniform vec2 u_texture_4_size;

// Environment map! Take a look at GLSL documentation to see how to
// sample from this.
uniform samplerCube u_texture_cubemap;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
  return texture(u_texture_4, uv).x;
}

void main() {
  // Your awesome shader here!
  float dU = (h(vec2(v_uv.x + 1/u_texture_2_size.x, v_uv.y)) - h(vec2(v_uv.x, v_uv.y))) * u_height_scaling * u_normal_scaling;
  float dV = (h(vec2(v_uv.x, v_uv.y + 1/u_texture_2_size.y)) - h(vec2(v_uv.x, v_uv.y))) * u_height_scaling * u_normal_scaling;
  vec3 no = vec3(-dU, -dV, 1);
  mat3 tbn = mat3(vec3(v_tangent), cross(vec3(v_normal), vec3(v_tangent)), vec3(v_normal));
  vec3 nd = tbn * no;
  vec3 l = u_light_pos - vec3(v_position);
  float kw = (1 + dot(vec3(nd), normalize(l))) / 2;
  vec3 cc = vec3(0.4, 0.4, 0.7);
  vec3 cw = vec3(0.8, 0.6, 0.6);
  out_color = vec4(kw * cw + (vec3(1.0, 1.0, 1.0) - kw) * cc, 1.0);
}
