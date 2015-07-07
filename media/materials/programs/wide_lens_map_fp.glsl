
uniform sampler2D RT;

void main()
{
	gl_FragColor = texture2D(RT,gl_TexCoord[0].xy)*vec4(gl_TexCoord[0].xy,0,1);
}