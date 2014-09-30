/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
/* Desc: Collision class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 */

#ifndef _PluginCOLLISION_HH_
#define _PluginCOLLISION_HH_

#include "gazebo/common/CommonTypes.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/plugin/PluginTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief Base class for all Plugin collisions.
    class GAZEBO_VISIBLE PluginCollision : public Collision
    {
      /// \brief Constructor.
      /// \param[in] _link Parent Link
      public: explicit PluginCollision(LinkPtr _parent);

      /// \brief Destructor.
      public: virtual ~PluginCollision();

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void Fini();

      /// \brief Set the encapsulated collsion object.
      /// \param[in] _collisionId Plugin id of the collision object.
      /// \param[in] _placeable True to make the object movable.
      public: void SetCollision(bool _placeable);

      /// \brief Get the Plugin collision class.
      /// \return The Plugin collision class.
      public: int GetCollisionClass() const;

      // Documentation inherited.
      public: virtual void OnPoseChange();

      // Documentation inherited.
      public: virtual void SetCategoryBits(unsigned int bits);

      // Documentation inherited.
      public: virtual void SetCollideBits(unsigned int bits);

      // Documentation inherited.
      public: virtual math::Box GetBoundingBox() const;

      /// \brief Similar to Collision::GetSurface, but provides dynamically
      ///        casted pointer to PluginSurfaceParams.
      /// \return Dynamically casted pointer to PluginSurfaceParams.
      public: PluginSurfaceParamsPtr GetPluginSurface() const;

      /// \brief Used when this is static to set the posse.
      private: void OnPoseChangeGlobal();

      /// \brief Used when this is not statis to set the posse.
      private: void OnPoseChangeRelative();

      /// \brief Empty pose change callback.
      private: void OnPoseChangeNull();

      /// \brief Function used to set the pose of the Plugin object.
      private: void (PluginCollision::*onPoseChangeFunc)();
    };
  }
}
#endif
