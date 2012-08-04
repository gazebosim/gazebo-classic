uniform sampler2D tex0;
uniform mat4 proj;
uniform vec4 ambientColor;
uniform vec3 farCorner;
uniform float farClipDistance;

void main()
{
  // Attribute 0: Normal+depth
  vec4 a0 = texture2D(tex0, gl_TexCoord[0].xy);

  // Clip fragment if depth is too close,
  // so the skybox can be rendered on the background
  // clip(a0.w-0.0001);
  if (a0.w < 0.0001)
    discard;

  // Calculate ambient colour of fragment
  gl_FragColor = vec4(ambientColor.xyz, 0.0);

  // Calculate depth of fragment
  vec3 viewPos = normalize(gl_TexCoord[1].xyz) * farClipDistance * a0.w;
  vec4 projPos = proj * vec4(viewPos, 1.0);
  gl_FragDepth = (projPos.z / projPos.w) * 0.5 + 0.5;
}
