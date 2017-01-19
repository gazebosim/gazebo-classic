/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_RENDERING_MARKERVISUAL_HH_
#define GAZEBO_RENDERING_MARKERVISUAL_HH_

#include <string>

#include <ignition/msgs.hh>

#include "gazebo/rendering/Visual.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    // Forward declare private data class
    class MarkerVisualPrivate;

    /// \cond
    /// \brief A marker visualization. The MarkerManager class should
    /// instantiate instances of this class.
    /// \sa MarkerManager
    class GZ_RENDERING_VISIBLE MarkerVisual : public Visual
    {
      /// \brief Constructor.
      /// \param[in] _name Name of the visual.
      /// \param[in] _vis Pointer to the parent Visual.
      public: MarkerVisual(const std::string &_name, VisualPtr _vis);

      /// \brief Destructor.
      public: virtual ~MarkerVisual();

      /// \brief Load the marker
      /// \param[in] _msg The marker message to load the visual from.
      public: void Load(const ignition::msgs::Marker &_msg);
      using Visual::Load;

      /// \brief Get the lifetime of the marker
      /// \return Life time of the marker in simulation time.
      public: common::Time Lifetime() const;

      // Documentation inherited
      public: virtual void Fini();

      /// \brief Populate a marker message.
      /// \param[in] _msg The message to populate.
      public: void FillMsg(ignition::msgs::Marker &_msg);

      /// \brief Add or modify a marker
      /// \param[in] _msg The message that defines what to add or modify
      private: void AddModify(const ignition::msgs::Marker &_msg);

      /// \brief Add or modify a dynamic renderable.
      /// \param[in] _msg The message that defines what to add or modify
      private: void DynamicRenderable(const ignition::msgs::Marker &_msg);

      /// \brief Add or modify movable text.
      /// \param[in] _msg The message that defines what to add or modify
      private: void Text(const ignition::msgs::Marker &_msg);

      /// \brief Private data pointer
      private: MarkerVisualPrivate *dPtr;
    };
    /// \endcond
  }
}
#endif
