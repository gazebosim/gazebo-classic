varying vec3 normal;
varying vec3 position;

uniform vec4 ambient;
uniform vec4 diffuse;
uniform vec4 specular;

void main()
{
  vec4 color = ambient * gl_LightSource[0].ambient;

  // normalize both input vectors
  vec3 n = normalize(normal);
  vec3 e = normalize(-position);

  vec3 lightDir = normalize(vec3(gl_LightSource[0].position));
  float NdotL = max(dot(normal, lightDir), 0.0);

  // if the vertex is lit compute the specular color
  if (NdotL> 0.0) {
      color += gl_LightSource[0].diffuse * diffuse * NdotL;
      // compute the half vector
      // vec3 halfVector = normalize(lightDir + e);
      vec3 halfVector = normalize(gl_LightSource[0].halfVector.xyz);
      // add specular
      float NdotH = max(dot(n, halfVector), 0.0);
      float shininess = 1.0;
      color += gl_LightSource[0].specular * specular * pow(NdotH, shininess);
  }
  gl_FragColor = color;
}
