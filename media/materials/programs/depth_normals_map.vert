varying vec3 oNormal;
void main()
{
  gl_Position = ftransform();
  // normal in view space
  oNormal = normalize(gl_NormalMatrix * gl_Normal);
}
