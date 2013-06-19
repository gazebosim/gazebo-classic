// The input texture, which is set up by the Ogre Compositor infrastructure.
uniform sampler2D RT;

// Other parameters are set in C++, via
// Ogre::GpuProgramParameters::setNamedConstant()

uniform float k1;
uniform float k2;
uniform float k3;
uniform float p1;
uniform float p2;

#define PI 3.14159265358979323846264

void main()
{
  // gl_FragColor = texture2D(RT, gl_TexCoord[0].xy);
  gl_FragColor = vec(1, 0, 0, 1)
}
