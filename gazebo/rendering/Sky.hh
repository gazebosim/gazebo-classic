/*
 * Copyright 2011 Nate Koenig
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
#ifndef _SKY_HH_
#define _SKY_HH_

#include "gazebo/sdf/sdf.hh"

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Subscriber.hh"
#include "gazebo/rendering/RenderTypes.hh"

namespace SkyX
{
  class SkyX;
  class BasicController;
}

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class Sky Sky.hh rendering/rendering.hh
    /// \brief Manages the Sky.
    ///
    /// This is the interface to SkX, which renders the sun, moon, stars,
    /// and clouds.
    class Sky
    {
      /// \brief Constructor.
      public: explicit Sky(ScenePtr _scene);

      /// \brief Destructor.
      public: virtual ~Sky();

      /// \brief Load the sky based on an sdf element.
      /// \param[in] _sdf SDF Sky element to load from.
      public: void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize.
      public: void Init();

      /// \brief Process a sky message.
      /// \param[in] _msg The message data.
      public: void ProcessSkyMsg(const msgs::Sky &_msg);

      /// \brief Add a camera to sky. This registers the camera's
      /// rendertarget with skyx.
      /// \param[in] _camera Pointer to a camera
      public: void AddCamera(CameraPtr _camera);

      /// \brief Helper function to setup the sky.
      private: void SetSky();

      /// \brief Sky message callback.
      /// \param[in] _msg The message data.
      private: void OnSkyMsg(ConstSkyPtr &_msg);

      /// \brief Pointer to the sky.
      private: SkyX::SkyX *skyx;

      /// \brief Controls the sky.
      private: SkyX::BasicController *skyxController;

      /// \brief Communication Node
      private: transport::NodePtr node;

      /// \brief Subscribe to sky updates.
      private: transport::SubscriberPtr skySub;

      /// \brief Scene SDF element.
      private: sdf::ElementPtr sdf;

      /// \brief Pointer to the scene.
      private: ScenePtr scene;
    };
  }
}
#endif
