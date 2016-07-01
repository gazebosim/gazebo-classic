#define MAX_RAND_SAMPLES 14
#define RADIUS 0.2125
#define NUM_BASE_SAMPLES 6

uniform mat4 ptMat;
uniform float far;
uniform sampler2D geomMap;
uniform sampler2D randMap;

/*const vec3 RAND_SAMPLES[6] = vec3[6](
  vec3(1.0, 0.0, 0.0),
  vec3(-1.0, 0.0, 0.0),
  vec3(0.0, 1.0, 0),
  vec3(0.0, -1.0, 0),
  vec3(0.0, 0.0, 1.0),
  vec3(0.0, 0.0, -1.0)
);
*/

//vec3 computeZ(vec2 xy)
//{
//  return vec3(xy, sqrt(1.0 - dot(xy, xy)));
//}

// for ps_3_0, we want to use tex2Dlod because it's faster
//ps_3_0 vec4 TEX2DLOD(sampler2D map, vec2 uv)
//{
//  return texture2DLod(map, vec4(uv.xy, 0.0, 0.0));
//}

vec4 TEX2DLOD(sampler2D map, vec2 uv)
{
  return texture2D(map, uv);
}


void main()
{

  // random normal lookup from a texture and expand to [-1..1]
  vec3 randN = TEX2DLOD(randMap, gl_TexCoord[0].xy * 24.0).xyz * 2.0 - 1.0;
  vec4 geom = TEX2DLOD(geomMap, gl_TexCoord[0].xy);

  float depth = geom.w;

  // IN.ray will be distorted slightly due to interpolation
  // it should be normalized here
  vec3 viewPos = normalize(gl_TexCoord[1].xyz) * depth;

  // by computing Z manually, we lose some accuracy under extreme angles
  // considering this is just for bias, this loss is acceptable
  vec3 viewNorm = geom.xyz; //computeZ(geom.yz);

  // accumulated occlusion factor
  float occ = 0.0;
  for (int i = 0; i < NUM_BASE_SAMPLES; ++i)
  {
    // reflected direction to move in for the sphere
    // (based on random samples and a random texture sample)
    // bias the random direction away from the normal
    // this tends to minimize self occlusion
    vec3 rr;
    if (i == 0)
      rr = vec3(1.0, 0.0, 0.0);
    else if (i==1)
      rr = vec3(-1.0, 0.0, 0.0);
    else if (i==2)
      rr = vec3(0.0, 1.0, 0);
    else if (i==3)
      rr = vec3(0.0, -1.0, 0);
    else if (i==4)
      rr = vec3(0.0, 0.0, 1.0);
    else if (i==5)
      rr = vec3(0.0, 0.0, -1.0);

    vec3 randomDir = reflect(rr, randN) + viewNorm;

    // move new view-space position back into texture space
    vec4 nuv = ptMat * vec4(viewPos.xyz + randomDir * RADIUS, 1.0);
    nuv.xy /= nuv.w;

    // compute occlusion based on the (scaled) Z difference
    float zd = clamp(far * (depth - TEX2DLOD(geomMap, nuv.xy).w), 0.0, 1.0);

    // this is a sample occlusion function, you can always play with
    // other ones, like 1.0 / (1.0 + zd * zd) and stuff
    occ += clamp(pow(1.0 - zd, 11.0) + zd, 0.0, 1.0);
  }

  occ /= 6.0;

  gl_FragColor = vec4(occ, occ, occ, 1.0);
}
