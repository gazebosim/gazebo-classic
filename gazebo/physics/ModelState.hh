/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_MODELSTATE_HH_
#define _GAZEBO_MODELSTATE_HH_

#include <vector>
#include <string>
#include <boost/regex.hpp>

#include <ignition/math/Vector3.hh>
#include "gazebo/math/Pose.hh"

#include "gazebo/physics/State.hh"
#include "gazebo/physics/LinkState.hh"
#include "gazebo/physics/JointState.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class ModelState ModelState.hh physics/physics.hh
    /// \brief Store state information of a physics::Model object
    ///
    /// This class captures the entire state of a Model at one
    /// specific time during a simulation run.
    ///
    /// State of a Model includes the state of all its child Links and
    /// Joints.
    class GZ_PHYSICS_VISIBLE ModelState : public State
    {
      /// \brief Default constructor.
      public: ModelState();

      /// \brief Constructor.
      ///
      /// Build a ModelState from an existing Model.
      /// \param[in] _model Pointer to the model from which to gather state
      /// info.
      /// \param[in] _realTime Real time stamp.
      /// \param[in] _simTime Sim time stamp.
      /// \param[in] _iterations Simulation iterations.
      public: ModelState(const ModelPtr _model, const common::Time &_realTime,
                  const common::Time &_simTime, const uint64_t _iterations);

      /// \brief Constructor.
      ///
      /// Build a ModelState from an existing Model.
      /// \param[in] _model Pointer to the model from which to gather state
      /// info.
      public: explicit ModelState(const ModelPtr _model);

      /// \brief Constructor
      ///
      /// Build a ModelState from SDF data
      /// \param[in] _sdf SDF data to load a model state from.
      public: explicit ModelState(const sdf::ElementPtr _sdf);

      /// \brief Destructor.
      public: virtual ~ModelState();

      /// \brief Load state from Model pointer.
      ///
      /// Build a ModelState from an existing Model.
      /// \param[in] _model Pointer to the model from which to gather state
      /// info.
      /// \param[in] _realTime Real time stamp.
      /// \param[in] _simTime Sim time stamp.
      /// \param[in] _iterations Simulation iterations.
      public: void Load(const ModelPtr _model, const common::Time &_realTime,
                  const common::Time &_simTime, const uint64_t _iterations);

      /// \brief Load state from SDF element.
      ///
      /// Load ModelState information from stored data in and SDF::Element
      /// \param[in] _elem Pointer to the SDF::Element containing state info.
      public: virtual void Load(const sdf::ElementPtr _elem);

      /// \brief Get the stored model pose.
      /// \return The math::Pose of the Model.
      public: const math::Pose &GetPose() const;

      /// \brief Get the stored model scale.
      /// \return The scale of the Model.
      public: const ignition::math::Vector3d &Scale() const;

      /// \brief Return true if the values in the state are zero.
      /// \return True if the values in the state are zero.
      public: bool IsZero() const;

      /// \brief Get the number of link states.
      ///
      /// This returns the number of Links recorded.
      /// \return Number of LinkState recorded.
      public: unsigned int GetLinkStateCount() const;

      /// \brief Get link states based on a regular expression.
      /// \param[in] _regex The regular expression.
      /// \return List of link states whose names match the regular
      /// expression.
      public: LinkState_M GetLinkStates(const boost::regex &_regex) const;

      /// \brief Get joint states based on a regular expression.
      /// \param[in] _regex The regular expression.
      /// \return List of joint states whose names match the regular
      /// expression.
      public: JointState_M GetJointStates(const boost::regex &_regex) const;

      /// \brief Get a link state by Link name
      ///
      /// Searches through all LinkStates. Returns the LinkState with the
      /// matching name, if any.
      /// \param[in] _linkName Name of the LinkState
      /// \return State of the Link.
      /// \throws common::Exception When _linkName is invalid.
      public: LinkState GetLinkState(const std::string &_linkName) const;

      /// \brief Return true if there is a link with the specified name.
      /// \param[in] _linkName Name of the LinkState.
      /// \return True if the link exists in the model.
      public: bool HasLinkState(const std::string &_linkName) const;

      /// \brief Get the link states.
      /// \return A map of link states.
      public: const LinkState_M &GetLinkStates() const;

      /// \brief Get the number of joint states.
      ///
      /// Returns the number of JointStates recorded.
      /// \return Number of JointStates.
      public: unsigned int GetJointStateCount() const;

      /// \brief Get a Joint state.
      ///
      /// Return a JointState based on a index, where index is between
      /// 0...ModelState::GetJointStateCount().
      /// \param[in] _index Index of a JointState.
      /// \return State of a Joint.
      /// \throws common::Exception When _index is out of range.
      public: JointState GetJointState(unsigned int _index) const;

      /// \brief Get a Joint state by Joint name.
      ///
      /// Searches through all JointStates. Returns the JointState with the
      /// matching name, if any.
      /// \param[in] _jointName Name of the JointState.
      /// \return State of the Joint.
      /// \throws common::Exception When _jointName is invalid.
      public: JointState GetJointState(const std::string &_jointName) const;

      /// \brief Get the joint states.
      /// \return A map of joint states.
      public: const JointState_M &GetJointStates() const;

      /// \brief Return true if there is a joint with the specified name.
      /// \param[in] _jointName Name of the Jointtate.
      /// \return True if the joint exists in the model.
      public: bool HasJointState(const std::string &_jointName) const;

      /// \brief Get the number of model states.
      ///
      /// This returns the number of nested model states recorded.
      /// \return Number of nested ModelState recorded.
      public: unsigned int NestedModelStateCount() const;

      /// \brief Get a model state by model name
      ///
      /// Searches through all nested model states. Returns the model state with
      /// the matching name, if any.
      /// \param[in] _modelName Name of the model state
      /// \return State of the Model.
      /// \throws common::Exception When _modelName is invalid.
      public: ModelState NestedModelState(const std::string &_modelName) const;

      /// \brief Return true if there is a nested model with the specified name.
      /// \param[in] _modelName Name of the model state.
      /// \return True if the model exists in this model state.
      public: bool HasNestedModelState(const std::string &_modelName) const;

      /// \brief Get the nested model states.
      /// \return A map of model names to model states.
      public: const ModelState_M &NestedModelStates() const;

      /// \brief Populate a state SDF element with data from the object.
      /// \param[out] _sdf SDF element to populate.
      public: void FillSDF(sdf::ElementPtr _sdf);

      /// \brief Set the wall time when this state was generated
      /// \param[in] _time The absolute clock time when the State
      /// data was recorded.
      public: virtual void SetWallTime(const common::Time &_time);

      /// \brief Set the real time when this state was generated
      /// \param[in] _time Clock time since simulation was stated.
      public: virtual void SetRealTime(const common::Time &_time);

      /// \brief Set the sim time when this state was generated
      /// \param[in] _time Simulation time when the data was recorded.
      public: virtual void SetSimTime(const common::Time &_time);

      /// \brief Set the simulation iterations when this state was generated
      /// \param[in] _iterations Simulation iterations when the data was
      /// recorded.
      public: virtual void SetIterations(const uint64_t _iterations);

      /// \brief Assignment operator
      /// \param[in] _state State value
      /// \return this
      public: ModelState &operator=(const ModelState &_state);

      /// \brief Subtraction operator.
      /// \param[in] _pt A state to substract.
      /// \return The resulting state.
      public: ModelState operator-(const ModelState &_state) const;

      /// \brief Addition operator.
      /// \param[in] _pt A state to substract.
      /// \return The resulting state.
      public: ModelState operator+(const ModelState &_state) const;

      /// \brief Stream insertion operator.
      /// \param[in] _out output stream.
      /// \param[in] _state Model state to output.
      /// \return The stream.
      public: inline friend std::ostream &operator<<(std::ostream &_out,
                  const gazebo::physics::ModelState &_state)
      {
        math::Vector3 q(_state.pose.rot.GetAsEuler());
        _out << std::fixed <<std::setprecision(3)
          << "<model name='" << _state.GetName() << "'>"
          << "<pose>"
          << _state.pose.pos.x << " "
          << _state.pose.pos.y << " "
          << _state.pose.pos.z << " "
          << q.x << " "
          << q.y << " "
          << q.z << " "
          << "</pose>";

        for (LinkState_M::const_iterator iter =
            _state.linkStates.begin(); iter != _state.linkStates.end();
            ++iter)
        {
          _out << iter->second;
        }

        for (const auto &ms : _state.modelStates)
        {
          _out << ms.second;
        }

        // Output the joint information
        // for (JointState_M::const_iterator iter =
        //     _state.jointStates.begin(); iter != _state.jointStates.end();
        //     ++iter)
        // {
        //   _out << iter->second;
        // }

        _out << "</model>";

        return _out;
      }

      /// \brief Pose of the model.
      private: math::Pose pose;

      /// \brief Scale of the model.
      private: ignition::math::Vector3d scale;

      /// \brief All the link states.
      private: LinkState_M linkStates;

      /// \brief All the joint states.
      private: JointState_M jointStates;

      /// \brief All the model states.
      private: ModelState_M modelStates;
    };
    /// \}
  }
}
#endif
