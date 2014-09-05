// The input texture, which is set up by the Ogre Compositor infrastructure.
uniform sampler2D RT;

// distortion coefficients
uniform float k1;
uniform float k2;
uniform float k3;
uniform float p1;
uniform float p2;
uniform vec2 center;
uniform vec3 scale;
uniform vec3 scaleIn;

vec2 warp(vec2 texCoord)
{

  vec2 normalized = (texCoord - center.xy) * scaleIn.xy;
  float rSq = normalized.x * normalized.x + normalized.y * normalized.y;

  // radial distortion: k1, k2, k3
  vec2 dist = normalized * ( 1.0 +
      k1 * rSq +
      k2 * rSq * rSq +
      k3 * rSq * rSq * rSq);

  // tangential distortion: p1, p2
  dist.x += p2 * (rSq + 2 * (normalized.x*normalized.x)) +
      2 * p1 * normalized.x * normalized.y;
  dist.y += p1 * (rSq + 2 * (normalized.y*normalized.y)) +
      2 * p2 * normalized.x * normalized.y;

  return center.xy + scale * dist;
}

void main()
{
  vec2 uv = warp(gl_TexCoord[0].xy);
  gl_FragColor = texture2D(RT, uv);
}
