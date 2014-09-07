// The input texture, which is set up by the Ogre Compositor infrastructure.
uniform sampler2D RT;
uniform sampler2D distortionMap;

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
  dist.x += p2 * (rSq + 2.0 * (normalized.x*normalized.x)) +
      2.0 * p1 * normalized.x * normalized.y;
  dist.y += p1 * (rSq + 2.0 * (normalized.y*normalized.y)) +
      2.0 * p2 * normalized.x * normalized.y;

  vec2 warped = center.xy + scale.xy * dist.xy;
  clamp(warped, 0.0, 1.0);
  return warped;
}

void main()
{
  vec2 uv = warp(gl_TexCoord[0].xy);
  //gl_FragColor = texture2D(RT, uv);
  gl_FragColor = texture2D(distortionMap, uv);
  //gl_FragColor = vec4(1.0, 0.0, 0.0, 0.0);
}
