uniform vec4 inv_shadow_map_size;
uniform vec4 derived_light_diffuse_color;
uniform vec4 derived_light_specular_color;
uniform float surface_shininess;
uniform vec4 shadow_depth_range;

uniform vec4 light_position_view_space;
uniform vec4 light_position_world_space;
uniform vec4 light_direction_view_space;
uniform vec4 light_attenuation;
uniform vec4 spotlight_params;

uniform float light_casts_shadows;

uniform sampler2D shadow_map;
uniform sampler2D diffuse_map;

varying vec3 vertex_world_view_pos;
varying vec3 vertex_world_pos;
varying vec3 vertex_world_norm;
varying vec4 vertex_light_pos;

//------------------------------------------------------------------------------
// A Simple blur function
vec4 Blur(sampler2D map, vec2 uv, const in vec2 offset, float steps, float adjust)
{
  float stepSize = offset.x;
  uv.xy -= vec2(stepSize * steps);

  vec4 total = vec4(0.0, 0.0, 0.0, 0.0);
  for (float x = 0.0; x < steps; x+=1.0)
    for (float y = 0.0; y < steps; y+=1.0)
      total +=
        texture2D(map, vec2(uv.xy + vec2(x * stepSize, y * stepSize))) + adjust;

  return total / (steps * steps);
}

//------------------------------------------------------------------------------
// Calculate the shadow factor
float ShadowPCF(in sampler2D shadow_map, in vec4 shadow_map_pos, 
                const in vec2 offset)
{
  // Old depth calc, using linear distance
  //float depth = (vertex_light_pos.z - shadow_depth_range.x) * 
  //              shadow_depth_range.w;
  
  // Get the shadow map position
  shadow_map_pos = shadow_map_pos / shadow_map_pos.w;
  vec2 uv = shadow_map_pos.xy;

  float depth_adjust = 0.000001;
  vec2 c = Blur(shadow_map, uv, offset, 2.0, depth_adjust).xy;

  // standard variance shadow mapping code
  float variance = min(max( c.y - (c.x * c.x), 0.0), 1.0);
  float m_d = c.x - shadow_map_pos.z;
  float p = variance / (variance + m_d * m_d);

  return smoothstep(0.4, 1.0, shadow_map_pos.z <= c.x ? 1.0 : p);
}

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

  //////////////////////////////////////////////////////////////////////////////
  // COMPUTE SHADOW CONTRIBUTION
  //shadow_factor = ShadowPCF();
  //if (light_casts_shadows)
  float shadow_factor = ShadowPCF(shadow_map, vertex_light_pos, inv_shadow_map_size.xy);

  gl_FragColor = color * shadow_factor;
}
