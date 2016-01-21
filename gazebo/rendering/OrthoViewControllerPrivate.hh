/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_ORTHOVIEWCONTROLLER_PRIVATE_HH_
#define _GAZEBO_ORTHOVIEWCONTROLLER_PRIVATE_HH_

namespace gazebo
{
  namespace rendering
  {
    /// \internal
    /// \brief OrthoViewController private data.
    class OrthoViewControllerPrivate
    {
      /// \brief Scale used for zooming within the orthographic view
      public: double scale;
    };
  }
}
#endif
