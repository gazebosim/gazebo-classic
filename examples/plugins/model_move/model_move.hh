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

#ifndef MODEL_MOVE_HH
#define MODEL_MOVE_HH 1

#include <gazebo/msgs/pose_animation.pb.h>
#include <vector>

namespace gazebo
{
  /// \class model_move.hh model_move.cc
  /// \brief A plugin to transport a model from point to point using
  /// pose animation.
  class ModelMove : public ModelPlugin
  {
    /// \brief constructor
    public: ModelMove();

    /// \brief Perform movement of the model
    /// \param[in] start starting point
    /// \param[in] end goal point
    /// \param[in,out] translation translation done before start
    private: void move(const math::Vector3 &start, const math::Vector3 &end,
                       math::Vector3 &translation);

    /// \brief Parse goals defined in the SDF
    /// \param[in] _sdf sdf pointer corresponding to goals element
    /// \return True if parsing was succesfull or not
    private: bool LoadGoalsFromSDF(const sdf::ElementPtr _sdf);

    /// \brief prepare the movement of the model.
    ///
    /// Initialize the animation and related variables, call the
    /// move method between each goal
    public: void initiateMove();

    /// \brief callback to run when recieve a path message
    /// \param [in] msg path message received to animate
    public: void getPathMsg(ConstPoseAnimationPtr &msg);

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
    private: math::Vector3 start_position;

    /// \brief Path to follow
    private: std::vector<math::Pose> path_goals;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelMove);
}
#endif
