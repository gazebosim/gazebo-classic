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
#ifndef GAZEBO_RENDERING_MARKER_MANAGER_HH_
#define GAZEBO_RENDERING_MARKER_MANAGER_HH_

#include <ignition/msgs.hh>
#include "gazebo/rendering/RenderTypes.hh"

namespace gazebo
{
  namespace rendering
  {
    // Forwared declare private data class.
    class MarkerManagerPrivate;

    /// \internal
    /// \brief Creates, deletes, and maintains marker visuals. Only the
    /// Scene class should instantiate and use this class.
    class MarkerManager
    {
      /// \brief Constructor
      public: MarkerManager();

      /// \brief Destructor
      public: virtual ~MarkerManager();

      /// \brief Subscribe to the ~/marker topic and connect to the
      /// PreRenderEvent.
      /// \param[in] _scene Pointer to the scene.
      /// \return True on success
      private: bool Init(ScenePtr _scene);

      /// \brief Update the markers. This function is called on
      /// the PreRender event.
      private: void OnPreRender();

      /// \brief Callback that receives marker messages.
      /// \param[in] _req The marker message.
      /// \param[out] _rep The response message.
      /// \param[out] _result True/false result.
      private: void OnMarkerMsg(const ignition::msgs::Marker &_req,
                   ignition::msgs::StringMsg &_rep, bool &_result);

      /// \brief Process a marker message.
      /// \param[in] _msg The message data.
      private: bool ProcessMarkerMsg(const ignition::msgs::Marker &_msg);

      private: void OnList(const ignition::msgs::StringMsg &_req,
                   ignition::msgs::Marker_V &_rep, bool &_result);

      /// \internal
      /// \brief Private data pointer
      private: MarkerManagerPrivate *dataPtr;

      /// Make sure the Scene can access Init()
      private: friend class Scene;
    };
  }
}
#endif
