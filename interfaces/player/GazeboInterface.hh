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
/* Desc: Generic Gazebo Device Inteface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 */

#ifndef GAZEBO_PLAYER_INTERFACE_HH
#define GAZEBO_PLAYER_INTERFACE_HH

#include <string>
#include "player.h"
#include "gazebo/transport/TransportTypes.hh"

/// \addtogroup player
/// \brief Base class for all the player interfaces
/// \{
// Forward declarations
class GazeboDriver;

/// \brief Base class for all the player interfaces
class GazeboInterface
{
  /// \brief Constructor
  public: GazeboInterface(player_devaddr_t _addr, GazeboDriver *_driver,
                          ConfigFile *_cf, int _section);

  /// \brief Destructor
  public: virtual ~GazeboInterface();

  /// \brief Handle all messages. This is called from GazeboDriver
  public: virtual int ProcessMessage(QueuePointer &_respQueue,
                                     player_msghdr_t *_hdr, void *_data) = 0;

  /// \brief Update this interface, publish new info.
  public: virtual void Update() = 0;

  /// \brief Open a SHM interface when a subscription is received.
  ///        This is called fromGazeboDriver::Subscribe
  public: virtual void Subscribe() = 0;

  /// \brief Close a SHM interface. This is called from
  ///        GazeboDriver::Unsubscribe
  public: virtual void Unsubscribe() = 0;

  /// \brief Address of the Player Device
  public: player_devaddr_t device_addr;

  /// \brief Driver instance that created this device
  public: GazeboDriver *driver;

  protected: static std::string worldName;

  protected: gazebo::transport::NodePtr node;
};

/// \}

#endif
