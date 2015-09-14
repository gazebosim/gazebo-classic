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

#ifndef _GAZEBO_MARKER_MANAGER_HH_
#define _GAZEBO_MARKER_MANAGER_HH_

namespace gazebo
{
  namespace rendering
  {
    // Forwared declare private data class.
    class MarkerManagerPrivate;

    /// \brief Creates, deletes, and maintains marker visuals.
    class MarkerManager
    {
      /// \brief Constructor
      public: MarkerManager();

      /// \brief Destructor
      public: virtual ~MarkerManager();

      /// \brief Subscribe to the ~/marker topic and connect to the
      /// PreRenderEvent.
      public: void Init();

      /// \brief Update the markers. This function is called on
      /// the PreRender event.
      private: void OnPreRender();

      /// \brief Callback that receives marker messages.
      /// \param[in] _msg The marker message.
      private: void OnMarkerMsg(ConstMarkerPtr &_msg);

      /// \brief Process a marker message.
      /// \param[in] _msg The message data.
      private: bool ProcessMarkerMsg(const msgs::Marker &_msg);

      /// \internal
      /// \brief Private data pointer
      private: MarkerManagerPrivate *dataPtr;
    };
  }
}
#endif
