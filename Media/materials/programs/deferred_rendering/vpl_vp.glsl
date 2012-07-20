uniform sampler2D RSM;
uniform mat4 World;
uniform mat4 View;
uniform mat4 ViewProj;
uniform mat4 InvShadowProjMatrix;
uniform float radius;

vs_3_0 vec4 getClipCoords(vec3 coords)
{
	return vec4(2.0 * (coords.x - 0.5),
             -2.0 * (coords.y - 0.5),
              2.0 * (coords.z-0.5), 1.0);
}

gp4vp vec4 getClipCoords(vec3 coords)
{
	return vec4(2.0 * (coords.x - 0.5), -2.0 * (coords.y - 0.5), coords.z, 1.0);
}

void main()
{
  vec3 RSMNormal;
  vec3 RSMFlux;

  vec4 RSMVal = texture2D(RSM, vec2(gl_MultiTexCoord0.xy));

  float attenuation = 1.0 / (1.0 + dot(RSMVal.r, RSMVal.r));

  vec4 prPos = InvShadowProjMatrix *
               getClipCoords(vec3(gl_MultiTexCoord0.xy, RSMVal.r));

  prPos.xyz /= prPos.w;
  vec3 RSMPos = prPos.xyz;

  // unpack the normal
  RSMNormal.x = RSMVal.g;
  float posZ = RSMVal.b > 1.0;
  RSMNormal.y = RSMVal.b - 2 * posZ;
  RSMNormal.z = 2.0 * (posZ - 0.5) * sqrt(RSMNormal.x * RSMNormal.x +
                                          RSMNormal.y * RSMNormal.y);

  RSMFlux.b = frac(RSMVal.a);
  RSMVal.a -= RSMFlux.b;
  RSMVal.a /= 255.0;
  RSMFlux.g = frac(RSMVal.a);
  RSMVal.a -= RSMFlux.g;
  RSMVal.a /= 255.0;
  RSMFlux.r = frac(RSMVal.a);
  RSMFlux *= attenuation;
  gl_TexCoord[3] = (View * vec4(RSMNormal, 0.0)).xyz;

  vec3 worldPos = gl_Vertex.xyz * radius * length(RSMFlux) *
    clamp(1000 * dot(normalize(gl_Vertex.xyz), normalize(RSMNormal)), 0.0, 1.0)
    + RSMPos;

  vec4 projPos = ViewProj * vec4(worldPos,1.0);
  gl_TexCoord[0] = projPos;
  gl_TexCoord[0].xyz /= gl_TexCoord[0].w;
  gl_TexCoord[1] = vec4(RSMFlux, 1.0);

  gl_Position = projPos;
  gl_TexCoord[2] = mul(View, vec4(RSMPos,1.0));
  gl_TexCoord[2].xyz /= gl_TexCoord[2].w;
}
