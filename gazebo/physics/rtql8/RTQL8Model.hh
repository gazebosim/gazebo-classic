/*
 * Copyright 2012 Open Source Robotics Foundation
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

#ifndef _RTQL8MODEL_HH_
#define _RTQL8MODEL_HH_

#include "gazebo/physics/rtql8/rtql8_inc.h"
#include "gazebo/physics/rtql8/RTQL8Types.hh"
#include "gazebo/physics/Model.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_rtql8 RTQL8 Physics
    /// \brief rtql8 physics engine wrapper
    /// \{

    /// \class RTQL8Model
    /// \brief RTQL8 model class
    class RTQL8Model : public Model
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent object.
      public: explicit RTQL8Model(BasePtr _parent);

      /// \brief Destructor.
      public: virtual ~RTQL8Model();

      /// \brief Load the model.
      /// \param[in] _sdf SDF parameters to load from.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize the model.
      public: virtual void Init();

      /// \brief Update the model.
      public: virtual void Update();

      /// \brief Finalize the model.
      public: virtual void Fini();

      // Documentation inherited.
      // public: virtual void Reset();

      /// \brief
      public: rtql8::dynamics::SkeletonDynamics* GetSkeletonDynamics() {
        return rtql8SkeletonDynamics;
      }

      /// \brief
      public: void SetCanonicalJoint(rtql8::kinematics::Joint* _joint) {
        rtql8CanonicalJoint = _joint;
      }

      /// \brief
      public: rtql8::kinematics::Joint* GetCanonicalJoint(void) const {
        return rtql8CanonicalJoint;
      }

      /// \brief
      public: RTQL8PhysicsPtr GetRTQL8Physics(void) const;

      /// \brief
      public: rtql8::simulation::World* GetRTQL8World(void) const;

      /// \brief
      protected: rtql8::dynamics::SkeletonDynamics* rtql8SkeletonDynamics;

      /// \brief Parent joint of the canonical link.
      /// When the canonical link of this model is free floating link, this link
      /// does not have the parent joint to the world. However, all RTQL8's link
      /// must have a parent joint. For free floating link, RTQL8 connect the
      /// link with a 6dof joint to the world.
      /// A) If the canonical link has its parent joint, then just store it in
      /// this member variable.
      /// B) If the canonical link does not have its parent joint, then create
      /// 6dof joint and store the joint in this member variable.
      protected: rtql8::kinematics::Joint* rtql8CanonicalJoint;
    };
    /// \}
  }
}
#endif
