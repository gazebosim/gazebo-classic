uniform vec4 derived_light_diffuse_color;
uniform vec4 derived_light_specular_color;
uniform float surface_shininess;

uniform vec4 light_position_view_space;
uniform vec4 light_direction_view_space;
uniform vec4 light_attenuation;
uniform vec4 spotlight_params;

varying vec3 vertex_world_view_pos;
varying vec3 vertex_world_norm;

void main()
{
  vec4 color = gl_FrontMaterial.emission;

  // Normalized fragment normal
  vec3 norm = normalize(vertex_world_norm);

  // Direction from the fragment to the light 
  vec3 light_dir_view = light_position_view_space.xyz - 
                        vertex_world_view_pos.xyz * light_position_view_space.w;

  // light_position_view_space.w == 0 for directional lights
  float light_dist = length(light_dir_view);
  light_dir_view = normalize(light_dir_view);

  float lambert_term = max( dot(norm, light_dir_view), 0.0 );

  if (lambert_term > 0.0) 
  {
    vec3 view = -normalize(vertex_world_view_pos.xyz);
    vec3 halfway = normalize( view + light_dir_view );
    float nDotH = dot(norm, halfway);

    // Light attenuation
    float atten = 1.0 / (light_attenuation.y + 
                         light_attenuation.z*light_dist + 
                         light_attenuation.w*light_dist*light_dist);    

    // Modify attenuation for spot lights
    if (!(spotlight_params.x == 1.0 && spotlight_params.y == 0.0 && 
          spotlight_params.z == 0.0 && spotlight_params.w == 1.0))
    {
      float rho = dot(-light_direction_view_space.xyz, light_dir_view);

      float fSpotE  = clamp((rho - spotlight_params.y) / 
          (spotlight_params.x - spotlight_params.y),0.0,1.0);

      atten *= pow(fSpotE, spotlight_params.z);
    }

    // Add diffuse component
    color += derived_light_diffuse_color * lambert_term * atten;

    // Add specular component
    color += derived_light_specular_color * 
             pow(clamp(nDotH, 0.0, 1.0), surface_shininess) * atten;
  }

  gl_FragColor = color;
}
