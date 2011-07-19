/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <algorithm>
#include <boost/lexical_cast.hpp>
#include "common/Material.hh"

using namespace gazebo;
using namespace common;


unsigned int Material::counter = 0;

std::string Material::ShadeModeStr[SHADE_COUNT] = {"FLAT", "GOURAUD", "PHONG"};

std::string Material::BlendModeStr[BLEND_COUNT] = {"ADD", "MODULATE", "REPLACE"};

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Material::Material()
{
  this->name = "gazebo_material_" + boost::lexical_cast<std::string>(counter++);
  this->blendMode = REPLACE;
  this->shadeMode= GOURAUD;
  this->transparency = 0;
  this->shininess = 0;
  this->ambient.Set(1,1,1,1);
  this->diffuse.Set(1,1,1,1);
}

////////////////////////////////////////////////////////////////////////////////
/// Create a material with a default color 
Material::Material(const Color &clr)
{
  this->name = "gazebo_material_" + boost::lexical_cast<std::string>(counter++);
  this->blendMode = REPLACE;
  this->shadeMode= GOURAUD;
  this->transparency = 0;
  this->shininess = 0;
  this->ambient = clr;
  this->diffuse = clr;
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Material::~Material()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Get the name of the material
std::string Material::GetName() const
{
  return this->name;
}

////////////////////////////////////////////////////////////////////////////////
// Set a texture image
void Material::SetTextureImage(const std::string &tex)
{
  this->texImage = tex;
}

////////////////////////////////////////////////////////////////////////////////
// Set a texture image with resource_path
void Material::SetTextureImage(const std::string &_tex,
                               const std::string &_resourcePath)
{
  this->texImage = _resourcePath + _tex;
}

////////////////////////////////////////////////////////////////////////////////
/// Get a thie texture image
std::string Material::GetTextureImage() const
{
  return this->texImage;
}


////////////////////////////////////////////////////////////////////////////////
/// Set the ambient color
void Material::SetAmbient(const Color &clr)
{
  this->ambient = clr;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the ambient color
Color Material::GetAmbient() const
{
  return this->ambient;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the diffuse color
void Material::SetDiffuse(const Color &clr)
{
  this->diffuse = clr;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the diffuse color
Color Material::GetDiffuse() const
{
  return this->diffuse;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the specular color
void Material::SetSpecular(const Color &clr)
{
  this->specular = clr;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the specular color
Color Material::GetSpecular() const
{
  return this->specular;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the emissive color
void Material::SetEmissive(const Color &clr)
{
  this->emissive = clr;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the emissive color
Color Material::GetEmissive() const
{
  return this->emissive;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the transparency percentage (0..1)
void Material::SetTransparency(float t)
{
  this->transparency = std::min(t, (float)1.0);
  this->transparency = std::max(this->transparency, (float)0.0);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the transparency percentage (0..1)
float Material::SetTransparency() const
{
  return this->transparency;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the shininess 
void Material::SetShininess(float s)
{
  this->shininess = s;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the shininess 
float Material::GetShininess() const
{
  return this->shininess;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the shininess 
float Material::GetTransparency() const
{
  return this->shininess;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the blending mode
void Material::SetBlendMode(BlendMode b)
{
  this->blendMode = b;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the blending mode
Material::BlendMode Material::GetBlendMode() const
{
  return this->blendMode;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the shading mode
void Material::SetShadeMode(ShadeMode s)
{
  this->shadeMode = s;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the shading mode
Material::ShadeMode Material::GetShadeMode() const
{
  return this->shadeMode;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the point size
void Material::SetPointSize(double size)
{
  this->pointSize = size;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the point size
double Material::GetPointSize() const
{
  return this->pointSize;
}
