uniform vec4 lightAttenuation;
uniform vec4 lightPositionOS;
uniform vec4 cameraPositionOS;
uniform vec4 spotParams;

varying vec4 diffuse, specular;
varying vec3 normal;
varying vec4 position;

void main()
{
  vec4 color = vec4 (0.0, 0.0, 0.0, 0.0);

  vec3 n, halfV;
  float NdotL, NdotHV;
  float att, spotEffect;
  float spotExponent = spotParams[2];

  vec3 aux = vec3 (gl_ModelViewMatrix * lightPositionOS - position);
  float toLightDist = length (aux);
  vec3 lightDir = normalize (aux);
  vec3 halfVector = normalize(vec3 (lightDir + normalize (vec3 (gl_ModelViewMatrix * cameraPositionOS - position))));

  n = normalize (normal);

  /* compute the dot product between normal and ldir */
  NdotL = max (dot (n,lightDir),0.0);

  if (gl_LightSource[0].spotCosCutoff >= 0.0 && NdotL > 0.0) {

    spotEffect = dot(normalize(gl_LightSource[0].spotDirection), -lightDir);
    if (spotEffect > gl_LightSource[0].spotCosCutoff)
    {
      spotEffect = pow(max(spotEffect, 0.0), spotExponent);

      att = spotEffect / (lightAttenuation.y +
          lightAttenuation.z * toLightDist +
          lightAttenuation.w * toLightDist * toLightDist);

      color += att * (diffuse * NdotL);


      NdotHV = max (dot (n, halfVector), 0.0);
      color += att * specular * pow (NdotHV,gl_FrontMaterial.shininess);
    }
  }   

  gl_FragColor = color;
} 
