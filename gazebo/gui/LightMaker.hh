/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_LIGHTMAKER_HH_
#define _GAZEBO_LIGHTMAKER_HH_

#include <string>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/gui/EntityMaker.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    class Light;
  }

  namespace gui
  {
    class GAZEBO_VISIBLE LightMaker : public EntityMaker
    {
      /// \brief Constructor
      public: LightMaker();

      // Documentation inherited
      public: void Start(const rendering::UserCameraPtr _camera);

      // Documentation inherited
      public: void Stop();

      /// \brief Initialize the light maker from an existing light in the scene.
      /// \param[in] _lightName Name of existing light in the scene.
      /// \return True if initialization is successful.
      public: bool InitFromLight(const std::string &_lightName);

      /// \brief Initialize the light maker.
      /// \return True if the light maker is initialized successfully.
      protected: virtual bool Init();

      // Documentation inherited
      protected: virtual void CreateTheEntity();

      // Documentation inherited
      protected: virtual ignition::math::Vector3d EntityPosition() const;

      // Documentation inherited
      protected: virtual void SetEntityPosition(
          const ignition::math::Vector3d &_pos);

      protected: int state;
      protected: msgs::Light msg;
      protected: transport::PublisherPtr lightPub;
      private: static unsigned int counter;
      protected: std::string lightTypename;

      /// \brief Pointer to the light being spawned.
      private: rendering::LightPtr light;
    };

    class GAZEBO_VISIBLE PointLightMaker : public LightMaker
    {
      public: PointLightMaker() : LightMaker()
              {
                this->msg.set_type(msgs::Light::POINT);
                this->msg.set_cast_shadows(false);
                this->lightTypename = "point";
              }
    };

    class GAZEBO_VISIBLE SpotLightMaker : public LightMaker
    {
      public: SpotLightMaker() : LightMaker()
              {
                this->msg.set_type(msgs::Light::SPOT);
                msgs::Set(this->msg.mutable_direction(),
                          math::Vector3(0, 0, -1));
                this->msg.set_cast_shadows(false);

                this->msg.set_spot_inner_angle(0.6);
                this->msg.set_spot_outer_angle(1.0);
                this->msg.set_spot_falloff(1.0);
                this->lightTypename  = "spot";
              }
    };

    class GAZEBO_VISIBLE DirectionalLightMaker : public LightMaker
    {
      public: DirectionalLightMaker() : LightMaker()
              {
                this->msg.set_type(msgs::Light::DIRECTIONAL);
                msgs::Set(this->msg.mutable_direction(),
                          math::Vector3(.1, .1, -0.9));
                this->msg.set_cast_shadows(true);

                this->lightTypename  = "directional";
              }
    };
  }
}
#endif
