#version 120

uniform sampler2D WarpTexture;
uniform vec2 LensCenter;
uniform vec2 ScreenCenter;
uniform vec2 Scale;
uniform vec2 ScaleIn;
uniform vec4 HmdWarpParam;
uniform vec4 ChromAbParam;

varying vec2 Texcoord;

void main(void)
{
	vec2 theta = (Texcoord - LensCenter) * ScaleIn; // Scales texture coordinates to [-1, 1]
	float rSq = theta.x * theta.x + theta.y * theta.y;
	vec2 theta1 = theta * ( HmdWarpParam.x +
							HmdWarpParam.y * rSq +
							HmdWarpParam.z * rSq * rSq +
							HmdWarpParam.w * rSq * rSq * rSq);
	// Detect whether blue texture coordinates are out of range since these will scaled out the furthest.
	vec2 thetaBlue = theta1 * (ChromAbParam.z + ChromAbParam.w * rSq);
	vec2 tcBlue = LensCenter + Scale * thetaBlue;

	if (any(bvec2(clamp(tcBlue, ScreenCenter - vec2(0.5, 0.5), ScreenCenter + vec2(0.5, 0.5)) - tcBlue))) {
		gl_FragColor = vec4(0, 0, 0, 0);
		return;
	}

	// Now do blue texture lookup.
	float blue = texture2D(WarpTexture, tcBlue).b;
	// Do green lookup (no scaling).
	vec2 tcGreen = LensCenter + Scale * theta1;
	float green = texture2D(WarpTexture, tcGreen).g;
	// Do red scale and lookup.
	vec2 thetaRed = theta1 * (ChromAbParam.x + ChromAbParam.y * rSq);
	vec2 tcRed = LensCenter + Scale * thetaRed;
	float red = texture2D(WarpTexture, tcRed).r;
	gl_FragColor = vec4(red, green, blue, 1);
}
