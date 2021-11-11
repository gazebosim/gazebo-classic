#version 120

uniform sampler2D tex;

uniform float width;
uniform float height;

varying vec4 point;

void main()
{
  vec3 color = 255.0f * texture2D(tex, vec2(gl_FragCoord.s / width , gl_FragCoord.t / height)).xyz;
  float rgb = color.b + color.g * 256.0f + color.r * 256.0f * 256.0f;
  gl_FragColor = vec4(point.x, -point.y, -point.z, rgb);
}
