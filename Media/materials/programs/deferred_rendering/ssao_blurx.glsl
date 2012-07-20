#define NUM_BLUR_SAMPLES 8

uniform vec4 invTexSize;
uniform sampler2D map;
uniform sampler2D geomMap;

//ps_3_0 vec4 TEX2DLOD(sampler2D map, vec2 uv)
//{
//  return texture2DLod(map, vec4(uv.xy, 0.0, 0.0));
//}

vec4 TEX2DLOD(sampler2D map, vec2 uv)
{
  return texture2D(map, uv);
}

// vec2 uv : TEXCOORD0,
void main()
{
  vec2 o = vec2(invTexSize.x, 0.0);
  vec4 sum = TEX2DLOD(map, gl_TexCoord[0].xy) * (9.0);
  float denom = 9.0;
  vec4 geom = TEX2DLOD(geomMap, gl_TexCoord[0].xy);

  for (int i = 1; i <= NUM_BLUR_SAMPLES; ++i)
  {
    vec2 nuv = gl_TexCoord[0].xy + o * float(i);
    vec4 nGeom = TEX2DLOD(geomMap, nuv);
    float coef = (9.0 - float(i)) * float(dot(geom.xyz, nGeom.xyz) > 0.9);
    sum += TEX2DLOD(map, nuv) * coef;
    denom += coef;
  }

  for (int i = 1; i <= 4; ++i)
  {
    vec2 nuv = gl_TexCoord[0].xy + o * -float(i);
    vec4 nGeom = TEX2DLOD(geomMap, nuv);
    float coef = (9.0 - float(i)) * float(dot(geom.xyz, nGeom.xyz) > 0.9);
    sum += TEX2DLOD(map, nuv) * coef;
    denom += coef;
  }

  gl_FragColor = sum / denom;
}
