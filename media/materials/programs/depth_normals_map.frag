varying vec3 oNormal;

void main()
{
  gl_FragColor = vec4(oNormal.x, -oNormal.y, -oNormal.z, 1.0);
}
