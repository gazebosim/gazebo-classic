// The input texture, which is set up by the Ogre Compositor infrastructure.
uniform sampler2D RT;

// distortion coefficients
uniform vec3 radial;
uniform vec3 tangential;
uniform vec2 center;
uniform vec3 scale;
uniform vec3 scaleIn;

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
  dist.x += tangential.y * (rSq + 2 * (normalized.x*normalized.x)) +
      2 * tangential.x * normalized.x * normalized.y;
  dist.y += tangential.x * (rSq + 2 * (normalized.y*normalized.y)) +
      2 * tangential.y * normalized.x * normalized.y;

  return center.xy + scale * dist;
}

void main()
{
  vec2 uv = warp(gl_TexCoord[0].xy);
  gl_FragColor = texture2D(RT, uv);
}
