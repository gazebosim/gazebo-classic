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
#ifndef _GAZEBO_MARKER_VISUAL_PRIVATE_HH_
#define _GAZEBO_MARKER_VISUAL_PRIVATE_HH_

#include <vector>
#include <memory>

#include "gazebo/rendering/MovableText.hh"
#include "gazebo/rendering/VisualPrivate.hh"

namespace gazebo
{
  namespace rendering
  {
    class DynamicLines;

    /// \brief Private data for the marker Visual class.
    class MarkerVisualPrivate : public VisualPrivate
    {
      /// \brief Renders line segments
      public: std::unique_ptr<DynamicLines> dynamicRenderable;

      /// \brief Renders text.
      public: std::unique_ptr<MovableText> text;

      /// \brief Mutex to protect the contact message.
      public: std::mutex mutex;

      /// \brief The last marker message received.
      public: msgs::Marker msg;

      /// \brief Lifetime of the marker
      public: common::Time lifetime;
    };
  }
}
#endif
