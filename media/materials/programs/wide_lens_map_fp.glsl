
uniform samplerCube envMap;

uniform int projectionType;
uniform float HFOV;

varying vec2 frag_pos;

float pi = 3.141592653;

float r = length(frag_pos);
float f = 1.0;

vec3 map(float th)
{
	return vec3(-sin(th)*frag_pos.x/r,sin(th)*frag_pos.y/r,cos(th));
}

vec3 mercator()
{
	float alpha = frag_pos.y*pi/2.0;
	float phi	= (-frag_pos.x)*pi + pi/2.0;

	vec3 tc = vec3(
		cos(phi)*cos(alpha),
		sin(alpha),
		sin(phi)*cos(alpha)
	);

	return tc;
}

vec3 gnomonical()
{
	return map(atan(r,f));
}

vec3 stereographic()
{
	return map(2.0*atan(r,2.0*f));
}

vec3 equidistant()
{
	return map(r/f);
}

vec3 equisolid()
{
	return map(2.0*asin(r/(2.0*f)));
}

vec3 orthographic()
{
	return map(asin(r/f));
}

vec3 fallback()
{
	return mercator();
}

void main()
{
	vec3 tc;

	if(projectionType == 0) tc = mercator();		else
	if(projectionType == 1) tc = gnomonical();		else
	if(projectionType == 2) tc = stereographic();	else
	if(projectionType == 3) tc = equidistant();		else
	if(projectionType == 4) tc = equisolid();		else
	if(projectionType == 5) tc = orthographic();	else
		tc = fallback();

	gl_FragColor = vec4(textureCube(envMap,tc).rgb,1);
}