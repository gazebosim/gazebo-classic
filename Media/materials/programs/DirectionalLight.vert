varying vec4 diffuse, ambient;
varying vec3 normal, lightDir, halfVector;

void main()
{
  // transform the normal into eye space
  normal = gl_NormalMatrix * gl_Normal;

  lightDir = normalize(vec3(gl_LightSource[1].position));

  halfVector = normalize(gl_LightSource[1].halfVector.xyz);

  diffuse = gl_FrontMaterial.diffuse * gl_LightSource[1].diffuse;
  ambient = gl_FrontMaterial.ambient * gl_LightSource[1].ambient;
  ambient += gl_LightModel.ambient * gl_FrontMaterial.ambient;

  gl_Position = ftransform();
}
