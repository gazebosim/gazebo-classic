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

#ifndef _GAZEBO_MODEL_MOVE_HH_
#define _GAZEBO_MODEL_MOVE_HH_

#include <gazebo/msgs/pose_animation.pb.h>
#include <vector>

namespace gazebo
{
  /// \brief A plugin to transport a model from point to point using
  /// pose animation.
  class ModelMove : public ModelPlugin
  {
    /// \brief Constructor
    public: ModelMove();

    /// \brief Perform movement of the model
    /// \param[in] _start starting point
    /// \param[in] _end goal point
    /// \param[in, out] _translation translation done before start
    private: void Move(const math::Vector3 &_start, const math::Vector3 &_end,
                       math::Vector3 &_translation);

    /// \brief Parse goals defined in the SDF
    /// \param[in] _sdf sdf pointer corresponding to goals element
    /// \return True if parsing was succesfull or not
    private: bool LoadGoalsFromSDF(const sdf::ElementPtr _sdf);

    /// \brief prepare the movement of the model.
    /// Initialize the animation and related variables, call the
    /// move method between each goal
    public: void InitiateMove();

    /// \brief Callback to run when recieve a path message.
    /// \param[in] _msg path message received to animate.
    public: void OnPathMsg(ConstPoseAnimationPtr &_msg);

    /// \brief Plugin Load function
    /// \param[in] _parent Model pointer to the model defining this plugin
    /// \param[in] _sdf pointer to the SDF of the model
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Pointer to the model that defines this plugin
    private: physics::ModelPtr model;

    /// \brief pointer to the move animation to perform
    private: gazebo::common::PoseAnimationPtr anim;

    /// \brief Transport node used to communicate with the transport system
    private: transport::NodePtr node;

    /// \brief Subscriber to get path messages
    private: transport::SubscriberPtr pathSubscriber;

    /// \brief Starting point of the path to follow
    private: math::Vector3 startPosition;

    /// \brief Path to follow
    private: std::vector<math::Pose> pathGoals;
  };
}
#endif
