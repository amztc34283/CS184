#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  
  // (Placeholder code. You will want to replace it.)
  vec3 l = u_light_pos - vec3(v_position);
  vec3 c = u_cam_pos - vec3(v_position);
  vec4 d = vec4(u_light_intensity * max(0.0, dot(vec3(v_normal), l / length(l))) / pow(length(l), 2.0), 1.0);
  vec3 half = l / length(l) + c / length(c);
  vec3 h = half / length(half);
  vec4 a = 0.8 * vec4(0.5,0.0,0.5,1.0);
  vec4 s = vec4(u_light_intensity * pow(max(0.0, dot(vec3(v_normal), h)), 50.0) / pow(length(l), 2.0), 1.0);
  out_color = a + d + s;
}

