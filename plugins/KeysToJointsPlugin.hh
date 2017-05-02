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

#include <string>
#include <vector>
#include <ignition/transport/Node.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/Node.hh>

namespace gazebo
{
  /// \brief Store information from SDF for each key
  struct KeyInfo
  {
    /// \brief Key ASCII value (reference: http://ascii.cl/)
    // cppcheck-suppress unusedStructMember
    int key;

    /// \brief Pointer to the joint controlled by this key.
    physics::JointPtr joint;

    /// \brief Possible target types: position, velocity, force.
    std::string type;

    /// \brief Increments for position, absolute values for velocity and
    /// force.
    // cppcheck-suppress unusedStructMember
    double scale;
  };

  /// \brief Control joints in a model based on keypress messages received.
  ///
  /// The plugin accepts multiple <map> elements:
  ///
  /// <map key='' joint='' type='' scale='' kp='' ki='' kd=''/>
  ///
  /// Where:
  ///
  /// * key: Key ASCII value (reference: http://ascii.cl/)
  /// * joint: Joint name
  /// * type: Available types: position, velocity, force.
  /// * scale: Slightly different according to type:
  ///     * position: scale is by how much the target position will increase or
  ///       decrease for each key press.
  ///     * velocity: The velocity target. This is not increased at each
  ///       keypress.
  ///     * force: The force to apply to the joint each time the key is pressed.
  ///
  /// Examples:
  ///
  /// 1. Decrease joint "revolute_joint_name"'s angle by 0.1 rad at every click
  /// on the "i" key:
  ///
  /// <map
  ///   key='105'
  ///   joint='revolute_joint_name'
  ///   scale='-0.1'
  ///   type='position'
  ///   kp='1000'
  ///   ki='0'
  ///   kd='10'
  /// />
  ///
  /// 2. Set joint "revolute_joint_name"'s angular velocity to 0.1 rad/s at
  /// every click on the "6" key:
  ///
  /// <map
  ///   key='54'
  ///   joint='revolute_joint_name'
  ///   scale='0.1'
  ///   type='velocity'
  ///   kp='200'
  ///   ki='0'
  ///   kd='0'
  /// />
  ///
  /// 3. Apply -1000 N to joint "prismatic_joint_name" at every click on the
  /// "u" key:
  ///
  /// <map
  ///   key='117'
  ///   joint='prismatic_joint_name'
  ///   scale='-1000'
  ///   type='force'
  /// />
  ///
  /// 4. Check the example world "simple_arm_teleop.world" for a demo.
  ///
  class GAZEBO_VISIBLE KeysToJointsPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: KeysToJointsPlugin();

    /// \brief Destructor
    public: ~KeysToJointsPlugin();

    // Documentation inherited
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback each time a key message is received.
    /// \param[in] _msg Keypress message.
    private: void OnKeyPress(ConstAnyPtr &_msg);

    /// \brief Stores information about each tracked key.
    private: std::vector<KeyInfo> keys;

    /// \brief Pointer to model.
    private: physics::ModelPtr model;

    /// \brief Node for communication.
    private: transport::NodePtr node;

    /// \brief Subscribe to keyboard messages.
    private: transport::SubscriberPtr keyboardSub;

    // Place ignition::transport objects at the end of this file to
    // guarantee they are destructed first.

    /// \brief Ignition node for communication.
    private: ignition::transport::Node nodeIgn;
  };
}
#endif

