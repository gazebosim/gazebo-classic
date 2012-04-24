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

/* Desc: A Light
 * Author: Nate Koenig
 * Date: 15 July 2003
 */

#ifndef LIGHT_HH
#define LIGHT_HH

#include <string>
#include <iostream>

#include "common/Event.hh"
#include "common/Color.hh"

#include "sdf/sdf.h"

namespace Ogre
{
  class Light;
  class SceneManager;
}

namespace gazebo
{
  namespace rendering
  {
    class Visual;
    class Scene;
    class DynamicLines;

    /// \addtogroup gazebo_rendering
    /// \{
    /// \brief Wrapper around an ogre light source
    class Light
    {
      /// \brief Constructor
      public: Light(Scene *scene);

      /// \brief Destructor
      public: virtual ~Light();

      /// \brief Load the light using a set of SDF parameters
      public: void Load(sdf::ElementPtr _sdf);

      /// \brief Load the light using default parameters
      public: void Load();

      /// \brief Load from a light message
      public: void LoadFromMsg(ConstLightPtr &msg);

        /// \brief Set the name of the visual
      public: void SetName(const std::string &name);

      /// \brief Get the name of the visual
      public: std::string GetName() const;
             
      /// \brief Get the type of the light 
      public: std::string GetType() const;

      /// \brief Set the position of the light
      public: void SetPosition(const math::Vector3 &p);

      /// \brief Set whether this entity has been selected by the user through
      ///        the gui
      public: virtual bool SetSelected(bool s);

      // \brief Toggle light visual visibility
      public: void ToggleShowVisual();

      /// \brief Set whether to show the visual
      public: void ShowVisual(bool s);

      /// \brief Set the light type
      public: void SetLightType(const std::string &type);

      /// \brief Set the diffuse
      public: void SetDiffuseColor(const common::Color &color);

      /// \brief Get the diffuse color
      public: common::Color GetDiffuseColor() const;

      /// \brief Set the specular color
      public: void SetSpecularColor(const common::Color &color);

      /// \brief Set the direction
      public: void SetDirection(const math::Vector3 &dir);

      /// \brief Get the direction
      public: math::Vector3 GetDirection() const;

      /// \brief Set the attenuation
      public: void SetAttenuation(double constant, double linear,
                                  double quadratic);

      /// \brief Set the spot light inner angle
      public: void SetSpotInnerAngle(const double &angle);

      /// \brief Set the spot light outter angle
      public: void SetSpotOuterAngle(const double &angle);

      /// \brief Set the spot light falloff
      public: void SetSpotFalloff(const double &angle);

      /// \brief Set the range
      public: void SetRange(const double &range);

      /// \brief Set cast shadowsj
      public: void SetCastShadows(const bool &cast);

      /// \private Helper node to create a visual representation of the light
      private: void CreateVisual();

      protected: virtual void OnPoseChange() {}
      /// The OGRE light source
      private: Ogre::Light *light;

      private: Visual *visual;
      private: DynamicLines *line;

      private: sdf::ElementPtr sdf;

      private: event::ConnectionPtr showLightsConnection;
      private: static unsigned int lightCounter;
      private: Scene *scene;
    };
    /// \}
  }
}
#endif


