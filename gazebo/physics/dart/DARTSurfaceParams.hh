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

#ifndef _GAZEBO_DARTSURFACEPARAMS_HH_
#define _GAZEBO_DARTSURFACEPARAMS_HH_

#include <sdf/sdf.hh>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/SurfaceParams.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics_dart
    /// \{

    /// Forward declare private data class
    class DARTSurfaceParamsPrivate;

    /// \brief Data structure containing DART surface parameters.
    /// Updating the parameters in this class doesn't update the
    /// actual DART objects.
    class GZ_PHYSICS_VISIBLE DARTSurfaceParams : public SurfaceParams
    {
      /// \brief Constructor.
      public: DARTSurfaceParams();

      /// \brief Destructor.
      public: virtual ~DARTSurfaceParams();

      /// \brief Load the contact params.
      /// \param[in] _sdf SDF values to load from.
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void FillMsg(msgs::Surface &_msg);

      // Documentation inherited.
      public: virtual void ProcessMsg(const msgs::Surface &_msg);

      // Documentation inherited.
      public: virtual FrictionPyramidPtr FrictionPyramid() const;

      /// \internal
      /// \brief Pointer to private data
      private: DARTSurfaceParamsPrivate *dataPtr;
    };
    /// \}
  }
}
#endif
