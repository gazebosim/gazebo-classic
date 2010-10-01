uniform vec3 derived_light_diffuse_color;
uniform vec4 derived_light_specular_color;
uniform float surface_shininess;
uniform vec4 light_position_view_space;
uniform vec4 light_attenuation;
uniform sampler2D diffuse_map;

varying vec3 vertex_world_view_pos;
varying vec3 vertex_world_norm;

void main()
{
  float specular = 0.0;

  // Normalized fragment normal
  vec3 norm = normalize(vertex_world_norm);

  // Direction from the fragment to the light 
  vec3 lightDirView = light_position_view_space.xyz - vertex_world_view_pos.xyz * light_position_view_space.w;

  // light_position_view_space.w == 0 for directional lights
  float lightDist = length(lightDirView);
  lightDirView = normalize(lightDirView);

  float lambertTerm = max( dot(norm, lightDirView), 0.0 );

  //////////////////////////////////////////////////////////////////////////////
  // COMPUTE DIFFUSE CONTRIBUTION
  vec4 diffuse_tex = texture2D(diffuse_map, gl_TexCoord[0].st);
  vec4 diffuse_contrib = vec4(derived_light_diffuse_color * diffuse_tex.rgb * 
                             lambertTerm,1.0);


  //////////////////////////////////////////////////////////////////////////////
  // COMPUTE SPOT AND SPECULAR COMPONENTS
  if (lambertTerm > 0.0 && lightDist <= light_attenuation.x) 
  {
    vec3 view = -normalize(vertex_world_view_pos.xyz);
    vec3 halfway = normalize( view + lightDirView );
    float nDotH = dot(norm, halfway);

    float fAtten = 1.0 / (light_attenuation.y + 
                          light_attenuation.z*lightDist + 
                          light_attenuation.w*lightDist*lightDist);    

    // Works for all light types
    specular = pow(clamp(nDotH, 0.0, 1.0), surface_shininess) * fAtten;
  }

  vec4 specular_contrib = specular * derived_light_specular_color;

  gl_FragColor = (diffuse_contrib + specular_contrib);
}
