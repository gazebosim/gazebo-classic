// Post shader: Light geometry material
#define LIGHT_POINT       1
#define LIGHT_SPOT        2
#define LIGHT_DIRECTIONAL 3

uniform sampler2D tex0;

#ifdef USE_MAT_PROPERTIES
uniform sampler2D tex1;
#endif

#if LIGHT_TYPE != LIGHT_POINT
uniform vec3 lightDir;
#endif

#if LIGHT_TYPE == LIGHT_SPOT
uniform vec4 spotParams;
#endif

#if LIGHT_TYPE != LIGHT_DIRECTIONAL
uniform float vpWidth;
uniform float vpHeight;
uniform vec3 farCorner;
uniform float flip;
#endif

#ifdef IS_SHADOW_CASTER
uniform mat4 invView;
uniform mat4 shadowViewProjMat;
uniform vec3 shadowCamPos;
uniform float shadowFarClip;
uniform sampler2D shadowTex;
#endif

uniform float farClipDistance;

// Attributes of light
uniform vec4 lightDiffuseColor;
uniform vec4 lightSpecularColor;
uniform vec4 lightFalloff;
uniform vec4 lightPos;

void checkShadow(sampler2D shadowMap, vec3 viewPos, mat4 invView,
	               mat4 shadowViewProj, float shadowFarClip)
{
	vec3 worldPos = (invView * vec4(viewPos, 1.0)).xyz;
	vec4 shadowProjPos = shadowViewProj * vec4(worldPos, 1.0);

	shadowProjPos /= shadowProjPos.w;

	vec2 shadowSampleTexCoord = shadowProjPos.xy;
	float shadowDepth = texture2D(shadowMap, shadowSampleTexCoord).x;

  if (shadowDepth < shadowProjPos.z - 0.005)
    discard;
}

//////////////////////////////////////////////////////////////////////////////
// Main shader section
//////////////////////////////////////////////////////////////////////////////	
void main()
{
  float NL = 0.0;
  float specular = 0.0;

	// None directional lights have some calculations to do in the beginning
  // of the pixel shader
#if LIGHT_TYPE != LIGHT_DIRECTIONAL
	vec4 projectionPos = gl_TexCoord[0] / gl_TexCoord[0].w;

	// -1 is because generally +Y is down for textures but up for the screen
  vec2 texCoord = vec2(projectionPos.x,
                       projectionPos.y * -1.0 * flip) * 0.5 + 0.5;

	// Texture coordinate magic, this compensates for jitter
	// texCoord = fixUV(texCoord, vec2(vpWidth, vpHeight));
	vec3 ray = vec3(projectionPos.x, projectionPos.y * flip, 1.0) * farCorner;
#else
  vec2 texCoord = gl_TexCoord[0].xy;
  vec3 ray = gl_TexCoord[1].xyz;
#endif

  // Attribute 0: Normal + depth
	vec4 a0 = texture2D(tex0, texCoord);

  // Distance from viewer (w)
	float distance = a0.w;
	vec3 normal = normalize(a0.xyz);

#ifdef USE_MAT_PROPERTIES
	// Attribute 1: Diffuse color + shininess
	vec4 a1 = texture2D(tex1, texCoord);

	// Attributes
	vec3 color = a1.xyz;
	float specularity = a1.w;
#endif

	// Calculate position of texel in view space
	vec3 viewPos = normalize(ray) * distance * farClipDistance;

	// Calculate light direction and distance
#if LIGHT_TYPE == LIGHT_DIRECTIONAL
  vec3 objToLightDir = -lightDir.xyz;
#else
  vec3 objToLightVec = lightPos.xyz - viewPos;
  float len_sq = dot(objToLightVec, objToLightVec);
  float len = sqrt(len_sq);
  vec3 objToLightDir = objToLightVec / len;
#endif

#ifdef IS_SHADOW_CASTER
  checkShadow(shadowTex, viewPos, invView, shadowViewProjMat, shadowFarClip);
#endif
	
	// Calculate diffuse color
	// vec3 total_light_contrib;

  // Lambertian term
	NL = max(0.0, dot(objToLightDir, normal));
                        // * lightDiffuseColor.xyz;

  vec3 viewDir;
  vec3 h;
#ifdef IS_SPECULAR
	// Calculate specular component
  viewDir = -normalize(viewPos);
  h = normalize(viewDir + objToLightDir);
  //specular = max(0.0, dot(viewDir, h));
  specular = max(0.0, dot(normal, h));
#ifdef USE_MAT_PROPERTIES
  specular = pow(specular, specularity);
#else
  specular = pow(specular, 30.0);
  //specular = pow(dot(normal, h), 30.0);
  //vec3 light_specular = pow(dot(normal, h), 32.0) * lightSpecularColor.xyz;
  //total_light_contrib += specularity * light_specular;
#endif
#endif

float attenuation = 1.0;
#if LIGHT_TYPE != LIGHT_DIRECTIONAL
#ifdef IS_ATTENUATED
  // clip(lightFalloff.x - len);
  //if (lightFalloff.x - len < 0.0)
  //  discard;
  
  // Calculate attenuation
  attenuation /= dot(lightFalloff.yzw, vec3(1.0, len, len_sq));
  
  // total_light_contrib /= attenuation;
#endif
#endif

#if LIGHT_TYPE == LIGHT_SPOT
  float spotlightAngle = clamp(dot(lightDir.xyz, -objToLightDir), 0.0, 1.0);
  float spotFalloff = clamp((spotlightAngle - spotParams.x) /
                      (spotParams.y - spotParams.x), 0.0, 1.0);

  attenuation *= (1.0 - spotFalloff);
  //total_light_contrib *= (1.0 - spotFalloff);
#endif

#ifdef USE_MAT_PROPERTIES
  // gl_FragColor = vec4(total_light_contrib * color, 0.0);
  gl_FragColor = vec4((NL * (lightDiffuseColor.xyz + specular * lightSpecularColor.xyz)) * color * attenuation, 1.0);
#else
  gl_FragColor = vec4(NL * attenuation * lightDiffuseColor.xyz,
                      specular * NL * attenuation);
#endif
}
