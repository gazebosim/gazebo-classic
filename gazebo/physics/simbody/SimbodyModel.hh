/*
 * Copyright 2013 Open Source Robotics Foundation
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
/* Desc: Base class for all models
 * Author: Nathan Koenig and Andrew Howard
 * Date: 8 May 2003
 */

#ifndef SIMBODYMODEL_HH_
#define SIMBODYMODEL_HH_

#include <string>
#include <map>
#include <vector>

#include "gazebo/physics/Model.hh"

namespace boost
{
  class recursive_mutex;
}

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class SimbodyModel SimbodyModel.hh physics/physics.hh
    /// \brief A model is a collection of links, joints, and plugins.
    class SimbodyModel : public Model
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent object.
      public: explicit SimbodyModel(BasePtr _parent);

      /// \brief Destructor.
      public: virtual ~SimbodyModel();

      /// \brief Load the model.
      /// \param[in] _sdf SDF parameters to load from.
      public: void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize the model.
      public: virtual void Init();
    };
    /// \}
  }
}
#endif
