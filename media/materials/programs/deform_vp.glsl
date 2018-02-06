varying vec3 normal;
varying vec3 position;
uniform float time;

// deform along vertex normal dir
vec3 deform(vec3 v)
{
  float z = 0.05 * sin((time+3.0*v.y)*4.0) * cos((time+3.0*v.x)*4.0);
  return v + gl_Normal * z;
}

void main()
{
  vec3 v = deform(gl_Vertex.xyz);

  // Note: need to deform normal but for simplicity keep the same normal
  vec3 n = gl_Normal;

  gl_Position = gl_ModelViewProjectionMatrix * vec4(v, 1.0);

  vec4 pos = gl_ModelViewMatrix * vec4(v, 1.0);
  position = pos.xyz / pos.w;
  normal = normalize(gl_NormalMatrix * n);
}

