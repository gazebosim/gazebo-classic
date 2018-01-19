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

#ifndef GAZEBO_PHYSICS_LIGHT_HH_
#define GAZEBO_PHYSICS_LIGHT_HH_

#include <memory>
#include "gazebo/physics/Entity.hh"

namespace gazebo
{
  namespace physics
  {
    class LightPrivate;
    class LightState;

    /// \addtogroup gazebo_physics
    /// \{

    /// \brief A light entity.
    class GZ_PHYSICS_VISIBLE Light : public Entity
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent object.
      public: explicit Light(BasePtr _parent);

      /// \brief Destructor
      public: virtual ~Light();

      /// \brief Initialize the light.
      public: void Init();

      /// \brief Update this light's parameters from a message.
      /// \param[in] _msg Message to process.
      public: void ProcessMsg(const msgs::Light &_msg);

      /// \brief Fill a light message with this light's parameters.
      /// \param[out] _msg Message to fill using this light's data.
      public: void FillMsg(msgs::Light &_msg);

      /// \brief Set the current light state.
      /// \param[in] _state State to set the light to.
      public: void SetState(const LightState &_state);

      // Documentation inherited
      public: void OnPoseChange();

      /// \brief Publish the pose.
      private: void PublishPose();

      /// \brief Indicate that the world pose should be recalculated.
      /// The recalculation will be done when Light::WorldPose is
      /// called.
      public: void SetWorldPoseDirty();

      // Documentation inherited.
      public: virtual const ignition::math::Pose3d &WorldPose() const;

      /// \brief Pointer to private data
      private: std::unique_ptr<LightPrivate> dataPtr;
    };
    /// \}
  }
}
#endif

