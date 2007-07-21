uniform vec4 lightDiffuse;
uniform vec4 lightSpecular;

varying vec4 diffuse, specular;
varying vec3 normal;
varying vec4 position;

void main()
{   
  /* first transform the normal into eye space and normalize the result */
  normal = normalize (gl_NormalMatrix * gl_Normal);
  position = gl_ModelViewMatrix * gl_Vertex;

  /* Compute the diffuse, ambient and globalAmbient terms */
  diffuse = gl_FrontMaterial.diffuse * lightDiffuse;
  specular = gl_FrontMaterial.specular * lightSpecular;

  gl_Position = ftransform();   
} 
