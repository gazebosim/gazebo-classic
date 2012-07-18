uniform sampler2D tex0;
uniform sampler2D tex1;
uniform float farClipDistance;

// Attributes of light
uniform vec4 lightDiffuseColor;
uniform vec4 lightSpecularColor;
uniform vec4 lightFalloff;
uniform vec3 lightPos;

uniform vec3 lightDir;

//varying vec2 texCoord;
//varying vec3 ray;

//////////////////////////////////////////////////////////////////////////////
// Helper function section
//////////////////////////////////////////////////////////////////////////////
//DirectX needs this to compensate for jitter
//ps_2_x vec2 fixUV(vec2 texCoord, vec2 texSize)
//{
//	return texCoord - (vec2(0.5,0.5)/texSize);
//}

//vec2 fixUV(vec2 texCoord, vec2 texSize)
//{
//  return texCoord;
// }

/*void checkShadow(sampler2D shadowMap, vec3 viewPos, mat4 invView,
	               mat4 shadowViewProj, float shadowFarClip,
                 vec3 shadowCamPos)
{
	vec3 worldPos = (invView * vec4(viewPos, 1)).xyz;
	float distanceFromLight = length(shadowCamPos - worldPos);
	vec4 shadowProjPos = shadowViewProj * vec4(worldPos,1);

	shadowProjPos /= shadowProjPos.w;

	vec2 shadowSampleTexCoord = shadowProjPos.xy;
	float shadowDepth = texture2D(shadowMap, shadowSampleTexCoord).r;
	float shadowDistance = shadowDepth * shadowFarClip;

	// clip(shadowDistance - distanceFromLight + 0.1);
	if (shadowDistance - distanceFromLight + 0.1 < 0.0)
    discard;
}*/

//////////////////////////////////////////////////////////////////////////////
// Main shader section
//////////////////////////////////////////////////////////////////////////////	
void main()
{
  // Attribute 0: Diffuse color + shininess
	vec4 a0 = texture2D(tex0, gl_TexCoord[0].xy);

  // Attribute 1: Normal + depth
	vec4 a1 = texture2D(tex1, gl_TexCoord[0].xy);

	// Attributes
	vec3 color = a0.xyz;
	float specularity = a0.w;

  // Distance from viewer (w)
	float distance = a1.w;
	vec3 normal = a1.xyz;

	// Calculate position of texel in view space
	vec3 viewPos = normalize(gl_TexCoord[1].xyz) * distance * farClipDistance;

	// Calculate light direction and distance
  vec3 objToLightDir = -lightDir.xyz;

	// Calculate diffuse color
	vec3 total_light_contrib;
	total_light_contrib = max(0.0, dot(objToLightDir, normal)) *
                        lightDiffuseColor.xyz;

  total_light_contrib = lightDiffuseColor.xyz;

  if (lightDiffuseColor.x  == 0.0 && lightDiffuseColor.y  == 0.0 &&
      lightDiffuseColor.z  == 0.0)
  {
    total_light_contrib = vec3(1.0, 1.0, 1.0);
  }

  gl_FragColor = vec4(total_light_contrib * color, 0.0);
}
