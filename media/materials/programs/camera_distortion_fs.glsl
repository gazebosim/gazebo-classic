// The input texture, which is set up by the Ogre Compositor infrastructure.
uniform sampler2D RT;

// distortion coefficients
uniform vec3 radial;
uniform vec3 tangential;
uniform vec2 center;
uniform vec3 scale;
uniform vec3 scaleIn;

bool equal(float a, float b)
{
  float eps = 0.000005;
  float c = a - b;
  if ( c*c < eps)
    return true;
  else return false;
}

vec2 warp(vec2 texCoord)
{

  vec2 normalized = (texCoord - center.xy) * scaleIn.xy;
  float rSq = normalized.x * normalized.x + normalized.y * normalized.y;

  // radial distortion: k1, k2, k3
  vec2 dist = normalized * ( 1.0 +
      radial.x * rSq +
      radial.y * rSq * rSq +
      radial.z * rSq * rSq * rSq);

  // tangential distortion: p1, p2
  dist.x += tangential.x * (rSq + 2 * (normalized.x*normalized.x)) +
      2 * tangential.y * normalized.x * normalized.y;
  dist.y += tangential.y * (rSq + 2 * (normalized.y*normalized.y)) +
      2 * tangential.x * normalized.x * normalized.y;

  return center.xy + scale * dist;
}

void main()
{
  vec2 uv = warp(gl_TexCoord[0].xy);

//  float tl = 0.13875;
//  float br = 0.86125;
  /*float tlx = 0.08125;
  float brx = 0.91875;
  float tly = 0.0833333;
  float bry = 0.916667;
  if ( equal(gl_TexCoord[0].x, tlx) || equal(gl_TexCoord[0].y, tly)
      || equal(gl_TexCoord[0].x, brx) || equal(gl_TexCoord[0].y, bry))
    gl_FragColor = vec4(1.0, 0.0, 0.0, 0.0);
  else*/ gl_FragColor = texture2D(RT, uv);
}
