/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_RAZER_HYDRA_HH_
#define _GAZEBO_RAZER_HYDRA_HH_

#include <gazebo/math/Filter.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/common/Plugin.hh>

namespace gazebo
{
  class GAZEBO_VISIBLE RazerHydra : public WorldPlugin
  {
    /// \brief Constructor.
    public: RazerHydra();

    /// \brief Destructor.
    public: virtual ~RazerHydra();

    // Documentation Inherited.
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Poll the hydra for input.
    /// \param[in] _lowPassCornerHz Filter frequency.
    /// \return true when there is a new update coming from the controller.
    private: bool Poll(float _lowPassCornerHz = 5.0);

    /// \brief Method executed in a separate thread to poll hydra for updates.
    private: void Run();

    /// \brief Update the hydra.
    /// \param[in] _info Update information.
    private: void Update(const common::UpdateInfo &_info);

    /// \brief Raw controller positions.
    private: int16_t rawPos[6];

    /// \brief Raw controller orientations.
    private: int16_t rawQuat[8];

    /// \brief Raw value of the buttons.
    private: uint8_t rawButtons[2];

    /// \brief Raw values of the analog joysticks.
    private: double rawAnalog[6];

    /// \brief Device file descriptor
    private: int hidrawFd;

    /// \brief Left and right controller positions.
    private: math::Vector3 pos[2];

    /// \brief Left and right controller orientations.
    private: math::Quaternion quat[2];

    /// \brief Left and right filtered positions.
    private: math::OnePoleVector3 filterPos[2];

    /// \brief Left and right filtered controller orientations.
    private: math::OnePoleQuaternion filterQuat[2];

    /// \brief Analog joysticks
    private: float analog[6];

    /// \brief Buttons that have been pressed.
    private: uint8_t buttons[14];

    /// \brief Estimate of the update period.
    private: math::OnePole<float> periodEstimate;

    /// \brief Time of the last poll cycle.
    private: common::Time lastCycleStart;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// \brief Mutex
    private: boost::mutex mutex;

    /// \brief Additional thread
    private: boost::thread *pollThread;

    /// \brief Use to stop the additional thread that the plugin uses.
    private: bool stop;

    /// \brief Gazebo communication node pointer.
    private: transport::NodePtr node;

    /// \brief Publisher pointer used to publish the messages.
    private: transport::PublisherPtr pub;
  };
}
#endif
