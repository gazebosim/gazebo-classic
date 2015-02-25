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

	  /// \brief
	  protected: dynamics::SkeletonDynamics* rtql8SkeletonDynamics;
    };
    /// \}
  }
}
#endif
