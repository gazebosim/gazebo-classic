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
#ifndef GAZEBO_RENDERING_ROAD2D_HH_
#define GAZEBO_RENDERING_ROAD2D_HH_

#include <string>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class Road Road.hh rendering/rendering.hh
    /// \brief Used to render a strip of road.
    class GZ_RENDERING_VISIBLE Road2d : public Visual
    {
      /// \brief Constructor
      public: Road2d();

      /// \brief Constructor.
      /// \param[in] _name Name of the road visual.
      /// \param[in] _parent Pointer to the parent Visual.
      public: Road2d(const std::string &_name, VisualPtr _parent);

      /// \brief Destructor
      public: virtual ~Road2d();

      /// \brief Load the visual using a road msg.
      /// \param[in] _msg Message containing road data.
      public: void Load(msgs::Road _msg);
      using Visual::Load;

      /// \brief Load the visual using a parent visual.
      /// \param[in] _parent Pointer to the parent visual.
      /// \sa see function that accepts msgs::Road parameter
      public: void Load(VisualPtr _parent) GAZEBO_DEPRECATED(8.0);
    };
    /// \}
  }
}
#endif
