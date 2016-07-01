/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#ifndef GAZEBO_GUI_LIGHTMAKER_HH_
#define GAZEBO_GUI_LIGHTMAKER_HH_

#include <memory>
#include <string>
#include <ignition/math/Vector3.hh>

#include "gazebo/msgs/light.pb.h"
#include "gazebo/gui/EntityMaker.hh"

namespace gazebo
{
  namespace gui
  {
    class LightMakerPrivate;

    /// \brief Used to insert a new light into the scene.
    class GZ_GUI_VISIBLE LightMaker : public EntityMaker
    {
      /// \brief Constructor
      public: LightMaker();

      /// \brief Destructor
      public: virtual ~LightMaker();

      // Documentation inherited
      public: void Start();

      // Documentation inherited
      public: void Stop();

      /// \brief Initialize the light maker from an existing light in the scene.
      /// \param[in] _lightName Name of existing light in the scene.
      /// \return True if initialization is successful.
      public: bool InitFromLight(const std::string &_lightName);

      // Documentation inherited
      public: virtual ignition::math::Vector3d EntityPosition() const;

      // Documentation inherited
      protected: virtual void SetEntityPosition(
          const ignition::math::Vector3d &_pos);

      /// \brief Initialize the light maker.
      /// \return True if the light maker is initialized successfully.
      protected: virtual bool Init();

      // Documentation inherited
      protected: virtual void CreateTheEntity();

      /// \brief Message that holds all the light information.
      protected: msgs::Light msg;

      /// \brief Type of the light being spawned.
      protected: std::string lightTypename;

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<LightMakerPrivate> dataPtr;
    };

    /// \brief Used to insert a new point light into the scene.
    class GZ_GUI_VISIBLE PointLightMaker : public LightMaker
    {
      /// \brief Constructor
      public: PointLightMaker();
    };

    /// \brief Used to insert a new spot light into the scene.
    class GZ_GUI_VISIBLE SpotLightMaker : public LightMaker
    {
      /// \brief Constructor
      public: SpotLightMaker();
    };

    /// \brief Used to insert a new directional light into the scene.
    class GZ_GUI_VISIBLE DirectionalLightMaker : public LightMaker
    {
      /// \brief Constructor
      public: DirectionalLightMaker();
    };
  }
}
#endif
