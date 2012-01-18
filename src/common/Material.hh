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
#ifndef MATERIAL_HH
#define MATERIAL_HH

#include <string>
#include <iostream>
#include "common/Color.hh"

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common Common
    /// \{
    /// \brief Encapsulates a description of a material
    class Material
    {
      public: enum ShadeMode {FLAT, GOURAUD, PHONG, BLINN, SHADE_COUNT};
      public: static std::string ShadeModeStr[SHADE_COUNT];

      public: enum BlendMode {ADD, MODULATE, REPLACE, BLEND_COUNT};
      public: static std::string BlendModeStr[BLEND_COUNT];

      /// \brief Constructor
      public: Material();

      /// \brief Destructor
      public: virtual ~Material();

      /// \brief Create a material with a default color
      public: Material(const Color &_clr);

      /// \brief Get the name of the material
      public: std::string GetName() const;

      /// \brief Set a texture image
      public: void SetTextureImage(const std::string &_tex);

      /// \brief Set a texture image
      public: void SetTextureImage(const std::string &tex,
                                   const std::string &resource_path);

      /// \brief Get a thie texture image
      public: std::string GetTextureImage() const;

      /// \brief Set the ambient color
      public: void SetAmbient(const Color &_clr);

      /// \brief Get the ambient color
      public: Color GetAmbient() const;

      /// \brief Set the diffuse color
      public: void SetDiffuse(const Color &_clr);

      /// \brief Get the diffuse color
      public: Color GetDiffuse() const;

      /// \brief Set the specular color
      public: void SetSpecular(const Color &_clr);

      /// \brief Get the specular color
      public: Color GetSpecular() const;

      /// \brief Set the emissive color
      public: void SetEmissive(const Color &_clr);

      /// \brief Get the emissive color
      public: Color GetEmissive() const;

      /// \brief Set the transparency percentage (0..1)
      public: void SetTransparency(float _t);

      /// \brief Get the transparency percentage (0..1)
      public: float SetTransparency() const;

      /// \brief Set the shininess
      public: void SetShininess(float _t);

      /// \brief Get the shininess
      public: float GetShininess() const;

      /// \brief Get the shininess
      public: float GetTransparency() const;

      /// \brief Set the blende factors. Will be interpreted as:
      ///        (texture * _srcFactor) + (scene_pixel * _dstFactor)
      public: void SetBlendFactors(double _srcFactor, double _dstFactor);

      /// \brief Get the blend factors
      public: void GetBlendFactors(double &_srcFactor, double &_dstFactor);

      /// \brief Set the blending mode
      public: void SetBlendMode(BlendMode _b);

      /// \brief Get the blending mode
      public: BlendMode GetBlendMode() const;

      /// \brief Set the shading mode
      public: void SetShadeMode(ShadeMode _b);

      /// \brief Get the shading mode
      public: ShadeMode GetShadeMode() const;

      /// \brief Set the point size
      public: void SetPointSize(double _size);

      /// \brief Get the point size
      public: double GetPointSize() const;

      /// \brief Set depth write
      public: void SetDepthWrite(bool _value);

      /// \brief Get depth write
      public: bool GetDepthWrite() const;

      /// \brief Set lighting enabled
      public: void SetLighting(bool _value);

      /// \brief Get lighting enabled
      public: bool GetLighting() const;

      /// \brief Ostream operator
      public: friend std::ostream &operator<<(std::ostream &_out,
                  const gazebo::common::Material &_m)
              {
                _out << "Material:\n";
                _out << "  Name: " << _m.name << "\n";
                _out << "  Texture: " << _m.texImage << "\n";
                _out << "  Ambient: " << _m.ambient << "\n";
                _out << "  Diffuse: " << _m.diffuse << "\n";
                _out << "  Specular: " << _m.specular << "\n";
                _out << "  Emissive: " << _m.emissive << "\n";
                _out << "  Transparency: " << _m.transparency << "\n";
                _out << "  Shininess: " << _m.shininess << "\n";
                _out << "  BlendMode: " << BlendModeStr[_m.blendMode] << "\n";
                _out << "  ShadeMode: " << ShadeModeStr[_m.shadeMode] << "\n";
                _out << "  DepthWrite: " << _m.depthWrite << "\n";
                return _out;
              }

      protected: std::string name;
      protected: std::string texImage;
      protected: Color ambient;
      protected: Color diffuse;
      protected: Color specular;
      protected: Color emissive;
      protected: float transparency;
      protected: float shininess;
      protected: double pointSize;

      protected: BlendMode blendMode;
      protected: ShadeMode shadeMode;

      private: static unsigned int counter;

      private: bool depthWrite;
      private: bool lighting;

      private: double srcBlendFactor, dstBlendFactor;
    };
    /// \}
  }
}
#endif


