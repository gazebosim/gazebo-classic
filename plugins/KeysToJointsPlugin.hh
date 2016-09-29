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

#ifndef GAZEBO_PLUGINS_KEYSTOJOINTSPLUGIN_HH_
#define GAZEBO_PLUGINS_KEYSTOJOINTSPLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/Node.hh>

namespace gazebo
{
  struct KeyInfo
  {
    int key;
    physics::JointPtr joint;
    double scale;
    std::string type;
  };

  /// \brief A plugin that simulates buoyancy of an object immersed in fluid.
  class GAZEBO_VISIBLE KeysToJointsPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: KeysToJointsPlugin();
    public: ~KeysToJointsPlugin();

    /// \brief Read the model SDF to compute volume and center of volume for
    /// each link, and store those properties in volPropsMap.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    private: void OnKeyPress(ConstAnyPtr &_msg);

    private: std::vector<KeyInfo> keys;
    private: physics::ModelPtr model;
    private: transport::NodePtr node;
    private: transport::SubscriberPtr keyboardSub;
  };
}
#endif
