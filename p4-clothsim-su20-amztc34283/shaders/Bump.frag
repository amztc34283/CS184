#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_3;
uniform vec2 u_texture_3_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
  return texture(u_texture_3, uv).x;
}

void main() {
  // YOUR CODE HERE

  // (Placeholder code. You will want to replace it.)
  float dU = (h(vec2(v_uv.x + 1/u_texture_3_size.x, v_uv.y)) - h(vec2(v_uv.x, v_uv.y))) * u_height_scaling * u_normal_scaling;
  float dV = (h(vec2(v_uv.x, v_uv.y + 1/u_texture_3_size.y)) - h(vec2(v_uv.x, v_uv.y))) * u_height_scaling * u_normal_scaling;
  vec3 no = vec3(-dU, -dV, 1);
  mat3 tbn = mat3(vec3(v_tangent), cross(vec3(v_normal), vec3(v_tangent)), vec3(v_normal));
  vec3 nd = tbn * no;
  vec3 l = u_light_pos - vec3(v_position);
  vec3 c = u_cam_pos - vec3(v_position);
  vec4 d = vec4(u_light_intensity * max(0.0, dot(vec3(nd), l / length(l))) / pow(length(l), 2.0), 1.0);
  vec3 half = l / length(l) + c / length(c);
  vec3 h = half / length(half);
  vec4 s = vec4(u_light_intensity * pow(max(0.0, dot(vec3(nd), h)), 50.0) / pow(length(l), 2.0), 1.0);
  out_color = d + s;
}

