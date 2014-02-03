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
#ifndef _GUITYPES_HH_
#define _GUITYPES_HH_

#include <boost/shared_ptr.hpp>

/// \file
/// \ingroup gazebo_gui
/// \brief default namespace for gazebo
namespace gazebo
{
  /// \brief GUI forward declarations and type defines
  namespace gui
  {
    class TopicView;
    class ImageView;
    class TextView;
    class LaserView;

    /// \def TopicViewPtr
    /// \brief Boost shared pointer to a TopicView object
    typedef boost::shared_ptr<TopicView> TopicViewPtr;

    /// \def ImageViewPtr
    /// \brief Boost shared pointer to a ImageView object
    typedef boost::shared_ptr<ImageView> ImageViewPtr;

    /// \def LaserViewPtr
    /// \brief Boost shared pointer to a LaserView object
    typedef boost::shared_ptr<LaserView> LaserViewPtr;

    /// \def TextViewPtr
    /// \brief Boost shared pointer to a TextView object
    typedef boost::shared_ptr<TextView> TextViewPtr;
  }
}

#endif
