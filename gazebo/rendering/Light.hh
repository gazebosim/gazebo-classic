/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#ifndef GAZEBO_RENDERING_LIGHT_HH_
#define GAZEBO_RENDERING_LIGHT_HH_

#include <string>
#include <iostream>
#include <sdf/sdf.hh>
#include <ignition/math/Color.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/common/Color.hh"
#include "gazebo/util/system.hh"

namespace Ogre
{
  class Light;
}

namespace gazebo
{
  namespace rendering
  {
    // Forward declare private data.
    class LightPrivate;

    /// \addtogroup gazebo_rendering
    /// \{

    /// \class Light Light.hh rendering/rendering.hh
    /// \brief A light source.
    ///
    /// There are three types of lights: Point, Spot, and Directional. This
    /// class encapsulates all three. Point lights are light light bulbs,
    /// spot lights project a cone of light, and directional lights are light
    /// sun light.
    class GZ_RENDERING_VISIBLE Light :
      public boost::enable_shared_from_this<Light>
    {
      /// \brief Constructor.
      /// \param[in] _scene Pointer to the scene that contains the Light.
      public: explicit Light(ScenePtr _scene);

      /// \brief Destructor
      public: virtual ~Light();

      /// \brief Load the light using a set of SDF parameters.
      /// \param[in] _sdf Pointer to the SDF containing the Light
      /// description.
      public: void Load(sdf::ElementPtr _sdf);

      /// \brief Load the light using default parameters.
      public: void Load();

      /// \brief Load from a light message.
      /// \param[in] _msg Containing the light information.
      public: void LoadFromMsg(ConstLightPtr &_msg);

      /// \brief Load from a light message.
      /// \param[in] _msg Message containing the light information.
      public: void LoadFromMsg(const msgs::Light &_msg);

      /// \brief Set the name of the visual.
      /// \param[in] _name Name of the light source.
      public: void SetName(const std::string &_name);

      /// \brief Get the name of the visual.
      /// \return The light's name.
      public: std::string Name() const;

      /// \brief Get the type of the light.
      /// \return The light type: "point", "spot", "directional".
      public: std::string Type() const;

      /// \brief Set the position of the light
      /// \param[in] _p New position for the light
      public: void SetPosition(const ignition::math::Vector3d &_p);

      /// \brief Get the position of the light
      /// \return The position of the light
      public: ignition::math::Vector3d Position() const;

      /// \brief Set the rotation of the light
      /// \param[in] _q New rotation for the light
      public: void SetRotation(const ignition::math::Quaterniond &_q);

      /// \brief Get the rotation of the light
      /// \return The rotation of the light
      public: ignition::math::Quaterniond Rotation() const;

      /// \brief Get the world pose of the light
      /// \return The world pose of the light
      public: ignition::math::Pose3d WorldPose() const;

      /// \brief Set whether this entity has been selected by the user through
      /// the gui.
      /// \param[in] _s Set to True when the light is selected by the user.
      public: virtual bool SetSelected(const bool _s);

      // \brief Toggle light visual visibility
      public: void ToggleShowVisual();

      /// \brief Set whether to show the visual
      /// \param[in] _s Set to true to draw a representation of the light.
      public: void ShowVisual(const bool _s);

      /// \brief Set whether the light will be visible
      /// \param[in] _s Set to true to make the light visible,
      /// false to turn the light off.
      public: void SetVisible(const bool _s);

      /// \brief Get whether the light is visible.
      /// \return True if the light is visible.
      public: bool Visible() const;

      /// \brief Set the light type.
      /// \param[in] _type The light type: "point", "spot", "directional"
      public: void SetLightType(const std::string &_type);

      /// \brief Get the light type.
      /// \return The light type: "point", "spot", "directional".
      public: std::string LightType() const;

      /// \brief Set the diffuse color
      /// \param[in] _color Light diffuse color.
      /// \deprecated See function which accepts ignition::math::Color.
      public: void SetDiffuseColor(const common::Color &_color)
          GAZEBO_DEPRECATED(9.0);

      /// \brief Set the diffuse color
      /// \param[in] _color Light diffuse color.
      public: void SetDiffuseColor(const ignition::math::Color &_color);

      /// \brief Get the diffuse color
      /// \return The light's diffuse color.
      public: ignition::math::Color DiffuseColor() const;

      /// \brief Set the specular color
      /// \param[in] _color The specular color
      /// \deprecated See function which accepts ignition::math::Color.
      public: void SetSpecularColor(const common::Color &_color)
          GAZEBO_DEPRECATED(9.0);

      /// \brief Set the specular color
      /// \param[in] _color The specular color
      public: void SetSpecularColor(const ignition::math::Color &_color);

      /// \brief Get the specular color
      /// \return  The specular color
      public: ignition::math::Color SpecularColor() const;

      /// \brief Set the direction
      /// \param[in] _dir Set the light's direction. Only applicable to spot and
      /// directional lights.
      public: void SetDirection(const ignition::math::Vector3d &_dir);

      /// \brief Get the direction
      /// \return The light's direction.
      public: ignition::math::Vector3d Direction() const;

      /// \brief Set the attenuation
      /// \param[in] _contant Constant attenuation
      /// \param[in] _linear Linear attenuation
      /// \param[in] _quadratic Quadratic attenuation
      public: void SetAttenuation(double _constant, double _linear,
                                  double _quadratic);

      /// \brief Get the spot light inner angle
      /// \return The inner angle in radians (or NaN if the light is not spot).
      public: double SpotInnerAngle() const;

      /// \brief Set the spot light inner angle
      /// \param[in] _angle Inner angle in radians
      public: void SetSpotInnerAngle(const double _angle);

      /// \brief Get the spot light outer angle
      /// \return The outer angle in radians (or NaN if the light is not spot).
      public: double SpotOuterAngle() const;

      /// \brief Set the spot light outer angle
      /// \param[in] _angle Outer angle in radians
      public: void SetSpotOuterAngle(const double _angle);

      /// \brief Get the spot light falloff
      /// \return The falloff value (or NaN if the light is not spot).
      public: double SpotFalloff() const;

      /// \brief Set the spot light falloff
      /// \param[in] _value Falloff value
      public: void SetSpotFalloff(const double _value);

      /// \brief Set the range
      /// \param[in] _range Rage of the light in meters.
      public: void SetRange(const double _range);

      /// \brief Set cast shadows
      /// \param[in] _cast Set to true to cast shadows.
      public: void SetCastShadows(const bool _cast);

      /// \brief Get cast shadows
      /// \return True if cast shadows.
      public: bool CastShadows() const;

      /// \brief Fill the contents of a light message.
      /// \param[out] _msg Message to fill.
      public: void FillMsg(msgs::Light &_msg) const;

      /// \brief Update a light source from a message.
      /// \param[in] _msg Light message to update from
      public: void UpdateFromMsg(ConstLightPtr &_msg);

      /// \brief Update a light source from a message.
      /// \param[in] _msg Light message to update from
      public: void UpdateFromMsg(const msgs::Light &_msg);

      /// \brief Clone the light with a new name
      /// \param[in] _name Name of the cloned light.
      /// \param[in] _scene Scene to contain the light.
      /// \return a clone of the light
      public: LightPtr Clone(const std::string &_name, ScenePtr _scene);

      /// \brief Get the id associated with this light
      /// \return Unique Light id
      public: uint32_t Id() const;

      /// \brief On pose change callback
      protected: virtual void OnPoseChange() {}

      /// \private Helper node to create a visual representation of the light
      private: void CreateVisual();

      /// \brief Update the ogre light source based on SDF values
      private: void Update();

      /// \brief Update SDF value based on a message.
      /// \param[in] _msg The light message to update from.
      private: void UpdateSDFFromMsg(const msgs::Light &_msg);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<LightPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
