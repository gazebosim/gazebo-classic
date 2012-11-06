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
/* Desc: A model state
 * Author: Nate Koenig
 */

#ifndef _MODEL_STATE_HH_
#define _MODEL_STATE_HH_

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

    /// \class ModelState ModelState.hh physics/physics.hh
    /// \brief Store state information of a physics::Model object
    ///
    /// This class captures the entire state of a Model at one
    /// specific time during a simulation run.
    ///
    /// State of a Model includes the state of all its child Links and
    /// Joints.
    class ModelState : public State
    {
      /// \brief Default constructor.
      public: ModelState();

      /// \brief Constructor.
      ///
      /// Build a ModelState from an existing Model.
      /// \param[in] _model Pointer to the model from which to gather state
      /// info.
      public: explicit ModelState(ModelPtr _model);

      /// \brief Destructor.
      public: virtual ~ModelState();

      /// \brief Load state from SDF element.
      ///
      /// Load ModelState information from stored data in and SDF::Element
      /// \param[in] _elem Pointer to the SDF::Element containing state info.
      public: virtual void Load(sdf::ElementPtr _elem);

      /// \brief Get the stored model pose.
      /// \return The math::Pose of the Model
      public: math::Pose GetPose() const;

      /// \brief Get the number of link states.
      ///
      /// This returns the number of Links recorded.
      /// \return Number of LinkState recorded.
      public: unsigned int GetLinkStateCount() const;

      /// \brief Get a link state.
      ///
      /// Get a Link State based on an index, where index is in the range of
      /// 0...ModelState::GetLinkStateCount
      /// \param[in] _index Index of the LinkState
      /// \return State of the Link
      public: LinkState GetLinkState(unsigned int _index) const;

      /// \brief Get a link state by Link name
      ///
      /// Searches through all LinkStates. Returns the LinkState with the
      /// matching name, if any.
      /// \param[in] _linkName Name of the LinkState
      /// \return State of the Link.
      public: LinkState GetLinkState(const std::string &_linkName) const;

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
      public: JointState GetJointState(unsigned int _index) const;

      /// \brief Get a Joint state by Joint name.
      ///
      /// Searches through all JointStates. Returns the JointState with the
      /// matching name, if any.
      /// \param[in] _jointName Name of the JointState.
      /// \return State of the Joint.
      public: JointState GetJointState(const std::string &_jointName) const;

      /// \brief Fill a State SDF element with state info.
      ///
      /// Stored state information into an SDF::Element pointer.
      /// \param[in] _elem Pointer to the SDF::Element which recieves the data.
      public: void FillStateSDF(sdf::ElementPtr _elem) const;

      /// \brief Update the state information using the specified model.
      public: void Update(ModelPtr _model);

      /// \brief Update a Model SDF element with this state info.
      ///
      /// Set the values in a Model's SDF::Element with the information
      /// stored in this instance.
      /// \param[in] _elem Pointer to a Models's SDF::Element.
      public: void UpdateModelSDF(sdf::ElementPtr _elem);

      /// \brief Return true if the values in the state are zero.
      /// \return True if the values in the state are zero.
      public: bool IsZero() const;

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
      public: friend std::ostream &operator<<(std::ostream &_out,
                                     const gazebo::physics::ModelState &_state)
      {
        _out << "<model name='" << _state.GetName() << "'>\n";
        _out << "<pose>" << _state.pose << "</pose>\n";

        /*
        for (std::vector<LinkState>::const_iterator iter =
            _state.linkStates.begin(); iter != _state.linkStates.end();
            ++iter)
        {
          _out << *iter;
        }

        for (std::vector<JointState>::const_iterator iter =
            _state.jointStates.begin(); iter != _state.jointStates.end();
            ++iter)
        {
          _out << *iter;
        }*/

        _out << "</model>\n";

        return _out;
      }

      /// Pose of the model
      private: math::Pose pose;

      /// All the link states
      private: std::vector<LinkState> linkStates;

      /// All the joint states
      private: std::vector<JointState> jointStates;
    };
    /// \}
  }
}
#endif
