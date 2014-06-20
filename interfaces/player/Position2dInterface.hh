/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
/* Desc: Position Interface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 */

#ifndef GAZEBO_POSITION2DINTERFACE_HH
#define GAZEBO_POSITION2DINTERFACE_HH

#include <string>

#include "gazebo/util/system.hh"
#include "GazeboInterface.hh"

/// \addtogroup player_iface
/// \{
/// \defgroup position2d_player Position2d Interface
/// \brief Position2d Player interface
/// \{
/// \brief Position2d Player interface
class GAZEBO_VISIBLE Position2dInterface : public GazeboInterface
{
  /// \brief Constructor
  public: Position2dInterface(player_devaddr_t _addr, GazeboDriver *_driver,
                              ConfigFile *_cf, int _section);

  /// \brief Destructor
  public: virtual ~Position2dInterface();

  /// \brief Handle all messages. This is called from GazeboDriver
  public: virtual int ProcessMessage(QueuePointer &_respQueue,
                                     player_msghdr_t *_hdr, void *_data);

  /// \brief Update this interface, publish new info.
  public: virtual void Update();

  /// \brief Open a SHM interface when a subscription is received.
  ///        This is called fromGazeboDriver::Subscribe
  public: virtual void Subscribe();

  /// \brief Close a SHM interface. This is called from
  ///        GazeboDriver::Unsubscribe
  public: virtual void Unsubscribe();

  /// \brief Process pose messages.
  /// \param[in] _msg A new pose message.
  private: void OnPoseMsg(ConstPose_VPtr &_msg);

  /// \brief Timestamp on last data update
  private: double datatime;

  private: gazebo::transport::PublisherPtr velPub;
  private: gazebo::transport::SubscriberPtr poseSub;
  private: std::string modelName;
};

/// \}
/// \}

#endif
