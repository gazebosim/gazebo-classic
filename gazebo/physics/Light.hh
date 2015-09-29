/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_PHYSICS_LIGHT_HH_
#define _GAZEBO_PHYSICS_LIGHT_HH_

#include "gazebo/physics/Entity.hh"
#include "gazebo/physics/Light.hh"

namespace boost
{
  class recursive_mutex;
}

namespace gazebo
{
  namespace physics
  {
    class LightState;

    /// \addtogroup gazebo_physics
    /// \{

    /// \class Light Light.hh physics/physics.hh
    /// \brief A light entity, which has no physics states, but is used
    /// by sensors and rendering.
    class GZ_PHYSICS_VISIBLE Light : public Entity
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent object.
      public: Light(BasePtr _parent);

      /// \brief Initialize the light.
      public: void Init();

      /// \brief Fill a light message.
      /// \param[out] _msg Message to fill using this light's data.
      public: void FillMsg(msgs::Light &_msg);

      /// \brief Update parameters from a light message.
      /// \param[in] _msg Message to process.
      public: void ProcessMsg(const msgs::Light &_msg);

      /// \brief Set the current light state.
      /// \param[in] _state State to set the light to.
      public: void SetState(const LightState &_state);

      /// \internal
      /// \brief Called when the pose is changed. Do not call this directly.
      public: virtual void OnPoseChange();

      /// \brief Get the light in message form.
      /// \return The light as a protobuf message.
      public: msgs::Light LightMsg() const;

      /// \brief Publish the pose.
      private: void PublishPose();

      /// \brief Light message container.
      protected: msgs::Light msg;
    };
    /// \}
  }
}
#endif

