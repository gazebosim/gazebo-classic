uniform sampler2D RSM;
uniform mat4 view;
uniform mat4 viewProj;

uniform mat4 World;
uniform mat4 InvShadowProjMatrix;
uniform float radius;

//vs_3_0 vec4 getClipCoords(vec3 coords)
vec4 getClipCoords(vec3 coords)
{
	return vec4(2.0 * (coords.x - 0.5),
             -2.0 * (coords.y - 0.5),
              2.0 * (coords.z-0.5), 1.0);
}

//gp4vp vec4 getClipCoords(vec3 coords)
//{
//	return vec4(2.0 * (coords.x - 0.5),
//	           -2.0 * (coords.y - 0.5),
//	            coords.z, 1.0);
//}

void main()
{
  vec3 RSMNormal;
  vec3 RSMFlux;

  vec4 RSMVal = texture2D(RSM, vec2(gl_MultiTexCoord0.xy));

  float attenuation = 1.0 / (1.0 + RSMVal.x * RSMVal.x);

  vec4 prPos = InvShadowProjMatrix *
               getClipCoords(vec3(gl_MultiTexCoord0.xy, RSMVal.x));

  prPos.xyz /= prPos.w;
  vec3 RSMPos = prPos.xyz;

  // unpack the normal
  RSMNormal.x = RSMVal.y;
  float posZ = float((RSMVal.z > 1.0));
  RSMNormal.y = RSMVal.z - (2.0 * posZ);
  RSMNormal.z = 2.0 * (posZ - 0.5) * sqrt(RSMNormal.x * RSMNormal.x +
                                          RSMNormal.y * RSMNormal.y);

  RSMFlux.z = fract(RSMVal.w);
  //RSMFlux.z = RSMVal.w - floor(RSMVal.w);

  RSMVal.w -= RSMFlux.z;
  RSMVal.w /= 255.0;

  RSMFlux.y = fract(RSMVal.w);
  //RSMFlux.y = RSMVal.w - floor(RSMVal.w);

  RSMVal.w -= RSMFlux.y;
  RSMVal.w /= 255.0;

  RSMFlux.x = fract(RSMVal.w);
  //RSMFlux.x = RSMVal.w - floor(RSMVal.w);
  
  RSMFlux *= attenuation;
  gl_TexCoord[3].xyz = (view * vec4(RSMNormal, 0.0)).xyz;

  vec3 worldPos = gl_Vertex.xyz * radius * length(RSMFlux) *
    clamp(1000.0 * dot(normalize(gl_Vertex.xyz), normalize(RSMNormal)),0.0, 1.0)
    + RSMPos;

  vec4 projPos = viewProj * vec4(worldPos, 1.0);
  gl_TexCoord[0] = projPos;
  gl_TexCoord[0].xyz /= gl_TexCoord[0].w;
  gl_TexCoord[1] = vec4(RSMFlux, 1.0);

  gl_Position = projPos;
  gl_TexCoord[2] = view * vec4(RSMPos, 1.0);
  gl_TexCoord[2].xyz /= gl_TexCoord[2].w;
}
