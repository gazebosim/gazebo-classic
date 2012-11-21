uniform sampler2D tex0;
uniform sampler2D tex1;
uniform mat4 proj;
uniform vec4 ambientColor;
uniform float farClipDistance;

void main()
{
  // Attribute 0: Normal+depth
  vec4 a0 = texture2D(tex0, gl_TexCoord[0].xy);

  // Attribute 1: Diffuse color+shininess
  vec4 a1 = texture2D(tex1, gl_TexCoord[0].xy);

	// Clip fragment if depth is too close, so the skybox can be rendered
  // on the background
  if (a0.w - 0.0001 < 0.0)
    discard;

  // Calculate ambient color of fragment
  gl_FragColor = ambientColor * vec4(a1.xyz, 0.0);

  // Calculate depth of fragment;
  vec3 viewPos = normalize(gl_TexCoord[1].xyz) * farClipDistance * a0.w;
  vec4 projPos = proj * vec4(viewPos, 1.0);

  gl_FragDepth = (projPos.z / projPos.w) * 0.5 + 0.5;
}
