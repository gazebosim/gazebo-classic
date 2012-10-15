#version 130
uniform sampler2D diffuse;
uniform vec3 spotParams;
uniform float farDistance;

in vec3 oViewPos;
in vec3 oNormal;
in vec3 oWorldPos;
in vec3 oProjPos;
in vec2 oTexcoord;

out vec4 fragColor;

vec4 packData(vec3 position, vec3 normal, vec3 flux, float depth)
{
  return vec4(
      depth,
      normal.x, //store x
      normal.y + 2.0 * float(normal.z > 0.0), //store y and z's sign
      trunc(flux.x * 255.0) * 255.0 + trunc(flux.y * 255.0) + flux.z
      );
}

void main()
{
  float depth = oProjPos.z;
  vec3 normal = normalize(oNormal);
  vec3 flux;

#ifdef DIRECTIONAL
  flux = texture2D(diffuse, oTexcoord).xyz;
#else
//  // #ifdef SPOT
//  // half spotlightAngle = clamp(dot(vec3(0.0, 0.0, 1.0), oViewPos), 0.0, 1.0);
//  // half spotFalloff = saturate((spotlightAngle - spotParams.x) / (spotParams.y - spotParams.x));
//  // flux = cLightColor;//vec3(1-spotFalloff);
//  // flux.b = 1;
#endif

  // #endif
  fragColor = packData(oWorldPos, normal, flux, depth);
}
