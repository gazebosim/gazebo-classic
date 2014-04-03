#version 120

uniform sampler2D WarpTexture;

uniform vec2 LensCenter;
uniform vec2 ScreenCenter;
uniform vec2 Scale;
uniform vec2 ScaleIn;
uniform vec4 HmdWarpParam;

varying vec2 Texcoord;

// Scales input texture coordinates for distortion.
vec2 HmdWarp(vec2 texCoord)
{
	vec2 theta = (texCoord - LensCenter) * ScaleIn; // Scales texture coordinates to [-1, 1]
	float rSq = theta.x * theta.x + theta.y * theta.y;
	vec2 rvector= theta * (	HmdWarpParam.x +
							HmdWarpParam.y * rSq +
							HmdWarpParam.z * rSq * rSq +
							HmdWarpParam.w * rSq * rSq * rSq);
	return LensCenter + Scale * rvector;
}

void main(void)
{
	vec2 tc = HmdWarp(Texcoord);
  gl_FragColor = texture2D(WarpTexture, tc);

//	if (any(bvec2(clamp(tc, ScreenCenter - vec2(0.5, 0.5), ScreenCenter + vec2(0.5, 0.5)) - tc))) {
//		gl_FragColor = vec4(0, 0, 0, 0);
//	} else {
//		gl_FragColor = texture2D(WarpTexture, tc);
//	}
}
