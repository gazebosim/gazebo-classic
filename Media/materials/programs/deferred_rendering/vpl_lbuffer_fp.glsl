uniform sampler2D GBuff;
uniform vec3 LightColor;
uniform vec4 viewportSize;
uniform float farClipDistance;
uniform mat4 invView;
uniform vec3 farCorner;
uniform float flip;
uniform float attenuation;
uniform float clampTo;

ps_2_x ps_3_0 vec2 fixUV(vec2 texCoord)
{
	return vec2(texCoord.x, -texCoord.y);
}

vec2 fixUV(vec2 texCoord)
{
	return texCoord;
}

void main()
{
	vec2 texcoord = gl_FradCoord.xy * viewportSize.zw;
	texcoord.y *= -flip;
	texcoord = fixUV(texcoord);
	vec4 normDepth = texture2D(GBuff, texcoord);
	vec3 normal = normalize(normDepth.xyz);
	vec3 ray = vec3(gl_TexCoord[0].x, flip * gl_TexCoord[1].y, 1.0) * farCorner;
	
	vec3 viewPos = normalize(ray) * normDepth.z * farClipDistance;
	vec3 lightToFrag = gl_TexCoord[2].xyz - viewPos;
	vec3 L = normalize(lightToFrag);
	float NL = max(0.0, dot(normal, L));

	float att = clamp(dot(gl_TexCoord[3], -normal), 0.0, 1.0);

	att *= 1.0 / (1.0 + attenuation * dot(lightToFrag, lightToFrag)); 
  gl_FragColor = vec4(clamp(gl_TexCoord[1].xyz * att * NL * LightColor, 0.0, clampTo), 0.0);

	// return vec4(clamp(gl_TexCoord[1].xyz*att*NL*LightColor,0,clampTo),0);
}
