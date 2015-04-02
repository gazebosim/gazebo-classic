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
/* Desc: Simulation Interface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 */

#ifndef GAZEBO_SIMULATION_INTERFACE_HH
#define GAZEBO_SIMULATION_INTERFACE_HH

#include <map>
#include <string>

#include "GazeboInterface.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/util/system.hh"

namespace boost
{
  class recursive_mutex;
}

/// \addtogroup player_iface
/// \{
/// \defgroup simulation_player Simulation Interface
/// Provides:
///  - PLAYER_SIMULATION_REQ_SET_POSE3D
///    - "set_pose3d"
///  - PLAYER_SIMULATION_REQ_SET_POSE2D
///    - "set_pose2d"
///  - PLAYER_SIMULATION_REQ_GET_POSE3D
///    - "get_pose"
///  - PLAYER_SIMULATION_REQ_GET_POSE2D
///    - "get_pose"
///  - PLAYER_SIMULATION_REQ_GET_PROPERTY
///    - "sim_time" returns double
///    - "real_time" returns double
///    - "pause_time" returns double
/// \brief The Simulation interface
/// \{
///

/// \brief The Simulation interface
class GAZEBO_VISIBLE SimulationInterface : public GazeboInterface
{
  /// \brief Constructor
  public: SimulationInterface(player_devaddr_t addr, GazeboDriver *driver,
                              ConfigFile *cf, int section);
  /// \brief Destructor
  public: virtual ~SimulationInterface();

  /// \brief Handle all messages. This is called from GazeboDriver
  public: virtual int ProcessMessage(QueuePointer &respQueue,
                                     player_msghdr_t *hdr, void *data);

  /// \brief Update this interface, publish new info.
  public: virtual void Update();

  /// \brief Open a SHM interface when a subscription is received.
  ///        This is called fromGazeboDriver::Subscribe
  public: virtual void Subscribe();

  /// \brief Close a SHM interface. This is called from
  ///        GazeboDriver::Unsubscribe
  public: virtual void Unsubscribe();

  private: void OnStats(ConstWorldStatisticsPtr &_msg);

  private: QueuePointer *responseQueue;

  private: player_simulation_pose3d_req_t pose3dReq;

  private: player_simulation_pose2d_req_t pose2dReq;
  private: static boost::recursive_mutex *mutex;

  private: gazebo::transport::SubscriberPtr statsSub;
  private: gazebo::transport::PublisherPtr modelPub;
  private: double simTime, realTime, pauseTime;
  private: bool paused;

  private: std::map<std::string, gazebo::math::Pose> entityPoses;
};

  /// \}
  /// \}
#endif
