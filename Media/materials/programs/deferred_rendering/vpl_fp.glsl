uniform sampler2D GBuff;
uniform sampler2D GBuff1;
uniform vec3 LightColor;
uniform vec4 viewportSize;
uniform float farClipDistance;
uniform vec4x4 invView;
uniform half3 farCorner;
uniform half flip;
uniform half attenuation;
uniform half clampTo;

ps_2_x ps_3_0 half2 fixUV(half2 texCoord)
{
	return vec2(texCoord.x, -texCoord.y);
}

half2 fixUV(half2 texCoord)
{
	return texCoord;
}

void main()
{
  vec2 texcoord = gl_FragCoord.xy * viewportSize.zw;
  texcoord.y *= -flip;
  texcoord = fixUV(texcoord);

  vec4 normDepth = texture2D(GBuff, vec2(texcoord.x, texcoord.y));
  vec3 albedo =    texture2D(GBuff1, vec2(texcoord.x, texcoord.y)).xyz;
  vec3 normal = normalize(normDepth.xyz);
  half3 ray = half3(gl_TexCoord[0].x, flip * gl_TexCoord[0].y, 1.0) * farCorner;

  vec3 viewPos = normalize(ray) * normDepth.z * farClipDistance;
  vec3 lightToFrag = gl_TexCoord[2].xyz - viewPos;
  vec3 L = normalize(lightToFrag);
  float NL = max(0.0, dot(normal, L));
  float att = 1.0;

  att *= 1.0 / (1.0 + attenuation * dot(lightToFrag, lightToFrag));

  gl_FragColor = vec4(clamp(albedo * gl_TexCoord[1].xyz * att * NL * LightColor,
                            0.0, clampTo), 1.0);

  // return half4(clamp(albedo*gl_TexCoord[1].rgb*att*NL*LightColor,0,clampTo),1);
