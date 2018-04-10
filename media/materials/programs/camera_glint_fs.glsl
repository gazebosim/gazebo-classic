//// This fragment shader will add glints to a rendered image. It's intended to
//// be instantiated via Ogre's Compositor framework so that we're operating on
//// a previously rendered image.  We're doing it as a shader for efficiency.
////
//// The CPU passes in the number of glints that are currently present, and
//// their 2D location in the image. Then we use polynomial values, also set
//// from the CPU (but determined by the user), to compute the effect that each
//// glint will have on the rendered image. The input to the polynomial is the distance
//// of each fragment to the glint. Polynomial coefficients which go
//// unused should be set to zero. The order of the highest polynomial value
//// can be set to avoid needless calls to pow()

// The input texture, which is set up by the Ogre Compositor infrastructure.
uniform sampler2D RT;

uniform vec3 sunlight;
uniform float tolerance;

varying vec4 point;
varying vec3 normal;

#define M_PI 3.1415926535897932384626433832795

bool isReflectingSunlight()
{
  float l = length(point.xyz);

  vec3 source = reflect(normalize(point.xyz), normalize(normal));

  float tolerance_internal = tolerance;
//  float tolerance_internal = 2.0*M_PI/180.0;
//  float tolerance_internal = 1.0 - cos(10.0*M_PI/180.0);

  float diff = dot(source, -sunlight) - 1.0;

  return (abs(diff) <= tolerance_internal);
}


void main()
{

  gl_FragColor = clamp(texture2D(RT, gl_TexCoord[0].xy), 0.0, 1.0);

  if(isReflectingSunlight())
  {
    gl_FragColor = vec4(1.0, 1.0, 1.0, 1.0);
  }
  else
  {
    gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
  }
}


