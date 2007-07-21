void main()
{
	gl_FrontColor = gl_LightModel.ambient * gl_FrontMaterial.ambient;
	
	// if you want to use more tex-coords, don't forget to assign them there
	gl_TexCoord[ 0 ] = gl_MultiTexCoord0;
	//gl_TexCoord[ 1 ] = gl_MultiTexCoord1;
	//gl_TexCoord[ 2 ] = gl_MultiTexCoord2;
	//gl_TexCoord[ 3 ] = gl_MultiTexCoord3;

	gl_Position = ftransform();
}
