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

#ifndef _SIMBODY_MODEL_HH_
#define _SIMBODY_MODEL_HH_

#include "gazebo/physics/Model.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class SimbodyModel SimbodyModel.hh physics/physics.hh
    /// \brief A model is a collection of links, joints, and plugins.
    class GAZEBO_VISIBLE SimbodyModel : public Model
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent object.
      public: explicit SimbodyModel(BasePtr _parent);

      /// \brief Destructor.
      public: virtual ~SimbodyModel();

      // Documentation inherited
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited
      public: virtual void Init();
    };
    /// \}
  }
}
#endif
