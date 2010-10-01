uniform sampler2D shadow_map_0;
uniform sampler2D shadow_map_1;
uniform sampler2D shadow_map_2;
uniform sampler2D diffuse_map;

uniform vec4 inv_shadow_map_size_0;
uniform vec4 inv_shadow_map_size_1;
uniform vec4 inv_shadow_map_size_2;

uniform vec3 derived_light_diffuse_color;
uniform vec4 derived_light_specular_color;
uniform float surface_shininess;

uniform vec4 shadow_depth_range_0;
uniform vec4 shadow_depth_range_1;
uniform vec4 shadow_depth_range_2;

uniform vec4 light_position_view_space;
uniform vec4 light_position_world_space;
uniform vec4 light_direction_view_space;
uniform vec4 light_attenuation;

uniform float light_casts_shadows;

uniform vec4 pssm_split_points;

varying vec3 vertex_world_view_pos;
varying vec3 vertex_world_norm;

varying vec4 vertex_light_pos_0;
varying vec4 vertex_light_pos_1;
varying vec4 vertex_light_pos_2;
varying float shadow_distance;

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

  float depth_adjust = 0.00001;
  vec2 c = Blur(shadow_map, uv, offset, 4.0, depth_adjust).xy;

  // standard variance shadow mapping code
  float variance = min(max( c.y - (c.x * c.x), 0.0), 1.0);
  float m_d = c.x - shadow_map_pos.z;
  float p = variance / (variance + m_d * m_d);

  return smoothstep(0.4, 1.0, shadow_map_pos.z <= c.x ? 1.0 : p);
}


  

void main()
{
  float spot = 1.0;
  float shadow_factor = 1.0;
  float specular = 0.0;

  // Normalized fragment normal
  vec3 norm = normalize(vertex_world_norm);

  // Direction from the fragment to the light 
  vec3 light_dir_view = light_position_view_space.xyz - 
                        vertex_world_view_pos.xyz * light_position_view_space.w;

  // light_position_view_space.w == 0 for directional lights
  float light_dist = length(light_dir_view);
  light_dir_view = normalize(light_dir_view);

  float lambertTerm = max( dot(norm, light_dir_view), 0.0 );

  //////////////////////////////////////////////////////////////////////////////
  // COMPUTE DIFFUSE CONTRIBUTION
  vec4 diffuse_tex = texture2D(diffuse_map, gl_TexCoord[0].st);
  vec4 diffuse_contrib = vec4(derived_light_diffuse_color * 
                             diffuse_tex.rgb * lambertTerm,1.0);


  //////////////////////////////////////////////////////////////////////////////
  // COMPUTE SPECULAR COMPONENT
  if (lambertTerm > 0.0 && light_dist <= light_attenuation.x) 
  {
    vec3 view = -normalize(vertex_world_view_pos.xyz);
    vec3 halfway = normalize( view + light_dir_view );
    float nDotH = dot(norm, halfway);

    float fAtten = 1.0 / (light_attenuation.y + 
                          light_attenuation.z*light_dist + 
                          light_attenuation.w*light_dist*light_dist);    

     // Works for all light types
    specular = pow(clamp(nDotH, 0.0, 1.0), surface_shininess) * fAtten;
  }

  //////////////////////////////////////////////////////////////////////////////
  // COMPUTE SHADOW CONTRIBUTION
  if (light_casts_shadows)
  {
    if (shadow_distance <= pssm_split_points.y)
    {
      shadow_factor = ShadowPCF(shadow_map_0, vertex_light_pos_0, 
                               inv_shadow_map_size_0.xy);
    }
    else if (shadow_distance <= pssm_split_points.z)
    {
      shadow_factor = ShadowPCF(shadow_map_1, vertex_light_pos_1, 
                               inv_shadow_map_size_1.xy);
    }
    else
    {
      shadow_factor = ShadowPCF(shadow_map_2, vertex_light_pos_2, 
                               inv_shadow_map_size_2.xy);
    }
  }

  vec4 specular_contrib = specular * derived_light_specular_color;

  gl_FragColor = (diffuse_contrib + specular_contrib) * shadow_factor;
}
