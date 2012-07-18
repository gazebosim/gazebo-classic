// Post shader: Light geometry material
#define LIGHT_POINT       1
#define LIGHT_SPOT        2
#define LIGHT_DIRECTIONAL 3

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform float farClipDistance;

// Attributes of light
uniform vec4 lightDiffuseColor;
uniform vec4 lightSpecularColor;
uniform vec4 lightFalloff;
uniform vec4 lightPos;

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
uniform sampler2D shadowTex;
uniform vec3 shadowCamPos;
uniform float shadowFarClip;
#endif


void checkShadow(sampler2D shadowMap, vec3 viewPos, mat4 invView,
	               mat4 shadowViewProj, float shadowFarClip,
#if LIGHT_TYPE == LIGHT_DIRECTIONAL
	vec3 shadowCamPos
#else
	float distanceFromLight
#endif
	)
{
	vec3 worldPos = (invView * vec4(viewPos, 1)).xyz;
#if LIGHT_TYPE == LIGHT_DIRECTIONAL
	float distanceFromLight = length(shadowCamPos - worldPos);
#endif
	vec4 shadowProjPos = shadowViewProj * vec4(worldPos,1);

	shadowProjPos /= shadowProjPos.w;

	vec2 shadowSampleTexCoord = shadowProjPos.xy;
	float shadowDepth = texture2D(shadowMap, shadowSampleTexCoord).r;
	float shadowDistance = shadowDepth * shadowFarClip;

	// clip(shadowDistance - distanceFromLight + 0.1);
	if (shadowDistance - distanceFromLight + 0.1 < 0.0)
    discard;
}

//////////////////////////////////////////////////////////////////////////////
// Main shader section
//////////////////////////////////////////////////////////////////////////////	
void main()
{
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

	// Attribute 0: Diffuse color + shininess
	vec4 a0 = texture2D(tex0, texCoord);

  // Attribute 1: Normal + depth
	vec4 a1 = texture2D(tex1, texCoord);

	// Attributes
	vec3 color = a0.xyz;
	float specularity = a0.w;

  // Distance from viewer (w)
	float distance = a1.w;
	vec3 normal = a1.xyz;

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
#if LIGHT_TYPE == LIGHT_DIRECTIONAL
  checkShadow(shadowTex, viewPos, invView, shadowViewProjMat,
              shadowFarClip, shadowCamPos);
#else
  checkShadow(shadowTex, viewPos, invView, shadowViewProjMat,
              shadowFarClip, len);
#endif
#endif
	
	// Calculate diffuse color
	vec3 total_light_contrib;

  // Lambertian term
	total_light_contrib = max(0.0, dot(objToLightDir, normal)) *
                        lightDiffuseColor.xyz;

#ifdef IS_SPECULAR
	// Calculate specular component
  vec3 viewDir = -normalize(viewPos);
  vec3 h = normalize(viewDir + objToLightDir);
  vec3 light_specular = pow(dot(normal, h), 32.0) * lightSpecularColor.xyz;
  total_light_contrib += specularity * light_specular;
#endif

#if LIGHT_TYPE != LIGHT_DIRECTIONAL
#ifdef IS_ATTENUATED
  // clip(lightFalloff.x - len);
  if (lightFalloff.x - len < 0.0)
    discard;
  
  // Calculate attenuation
  float attenuation = dot(lightFalloff.yzw, vec3(1.0, len, len_sq));
  
  total_light_contrib /= attenuation;
#endif
#endif

#if LIGHT_TYPE == LIGHT_SPOT
  float spotlightAngle = clamp(dot(lightDir.xyz, -objToLightDir), 0.0, 1.0);
  float spotFalloff = clamp((spotlightAngle - spotParams.x) /
                      (spotParams.y - spotParams.x), 0.0, 1.0);

  total_light_contrib *= (1.0 - spotFalloff);
#endif

  gl_FragColor = vec4(total_light_contrib * color, 0.0);
}
