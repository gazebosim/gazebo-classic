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
/* Desc: Laser Interface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 */

#ifndef LASERINTERFACE_HH
#define LASERINTERFACE_HH

#include <string>
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/util/system.hh"
#include "GazeboInterface.hh"

/// \addtogroup player_iface Interfaces
/// \{
/// \defgroup laser_player Laser Interface
/// \brief Plugin Player interface for a Gazebo laser
/// \{

/// \brief Plugin Player interface for a Gazebo laser
class GAZEBO_VISIBLE LaserInterface : public GazeboInterface
{
  /// \brief Constructor
  /// \param addr Plaer device address
  /// \param driver The Gazebo driver
  /// \param cf Player config file
  /// \param section Section of the config
  public: LaserInterface(player_devaddr_t addr, GazeboDriver *driver,
                         ConfigFile *cf, int section);

  /// \brief Destructor
  public: virtual ~LaserInterface();

  /// \brief Handle all messages. This is called from GazeboDriver
  /// \param respQueue Response queue
  /// \param hdr Message header
  /// \param data Pointer to the message data
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

  private: void OnScan(ConstLaserScanStampedPtr &_msg);

  private: std::string laserName;

  /// \brief Timestamp on last data update
  private: double datatime;

  private: int scanId;
  // private: player_laser_data_t data;
  private: player_laser_data_scanpose_t data;
  private: gazebo::transport::SubscriberPtr laserScanSub;
};
/// \}
/// \}
#endif
