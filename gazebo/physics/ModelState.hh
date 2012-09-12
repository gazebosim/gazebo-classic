/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: A model state
 * Author: Nate Koenig
 */

#ifndef MODEL_STATE_HH
#define MODEL_STATE_HH

#include <vector>
#include <string>

#include "physics/State.hh"
#include "physics/LinkState.hh"
#include "physics/JointState.hh"
#include "math/Pose.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief Keeps track of state of a model object
    class ModelState : public State
    {
      /// \brief Default constructor
      public: ModelState();

      /// \brief Constructor
      public: ModelState(ModelPtr _model);

      /// \brief Destructor
      public: virtual ~ModelState();

      /// \brief Load state from SDF element
      public: virtual void Load(sdf::ElementPtr _elem);

      /// \brief Get the model pose
      public: math::Pose GetPose() const;

      /// \brief Get the number of link states
      public: unsigned int GetLinkStateCount() const;

      /// \brief Get a link state
      public: LinkState GetLinkState(unsigned int _index) const;

      /// \brief Get a link state by model name
      public: LinkState GetLinkState(const std::string &_modelName) const;

      /// \brief Get the number of joint states
      public: unsigned int GetJointStateCount() const;

      /// \brief Get a model state
      public: JointState GetJointState(unsigned int _index) const;

      /// \brief Get a model state by model name
      public: JointState GetJointState(const std::string &_jointName) const;

      /// \brief Fill a State SDF element with state info
      public: void FillStateSDF(sdf::ElementPtr _elem);

      /// \brief Update a Model SDF element with this state info
      public: void UpdateModelSDF(sdf::ElementPtr _elem);

      private: math::Pose pose;
      private: std::vector<LinkState> linkStates;
      private: std::vector<JointState> jointStates;
    };
    /// \}
  }
}
#endif
