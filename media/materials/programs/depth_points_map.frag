#version 120

uniform sampler2D tex;

uniform float width;
uniform float height;

varying vec4 point;

void main()
{
  //vec3 color = 255 * texture2D(tex, vec2(gl_FragCoord.s / width , gl_FragCoord.t / height)).xyz;
  vec3 color = vec3(80, 0, 0);
  // int rgb = int(color.r) << 16 | int(color.g) << 8 | int(color.b);
  int rgb = 1;
  gl_FragColor = vec4(point.x, -point.y, -point.z, rgb);
}
