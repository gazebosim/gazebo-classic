/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include "gazebo/rendering/rendering.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
  /// \brief This plugin will generate 4 pictures of a model: perspective,
  /// top, left, front, back.
  class ModelPropShop : public SystemPlugin
  {
    /// \brief Destructor
    public: virtual ~ModelPropShop();

    /// \brief Load the plugin.
    public: void Load(int /*_argc*/, char ** /*_argv*/);

    /// \brief Initialize the plugin.
    private: void Init();

    /// \brief Update the plugin.
    private: void Update();

    /// \brief The connections.
    private: std::vector<event::ConnectionPtr> connections;

    /// \brief Node for communication.
    private: transport::NodePtr node;

    /// \brief Publisher used to stop the server.
    private: transport::PublisherPtr pub;

    /// \brief Pointer to the scene.
    private: rendering::ScenePtr scene;

    /// \brief Pointer to the camera.
    private: rendering::CameraPtr camera;
  };
}
