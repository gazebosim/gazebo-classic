/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#ifndef IRINTERFACE_HH
#define IRINTERFACE_HH

#include "GazeboInterface.hh"
#include "gazebo/util/system.hh"

namespace boost
{
  class recursive_mutex;
}

namespace libgazebo
{
/// \addtogroup player_iface Interfaces
/// \{
/// \defgroup laser_player Laser Interface
/// \brief Plugin Player interface for a Gazebo laser
/// \{
// Forward declarations
class IRIface;

/// \brief Plugin Player interface for a Gazebo laser
class GAZEBO_VISIBLE IRInterface : public GazeboInterface
{
  /// \brief Constructor
  /// \param addr Plaer device address
  /// \param driver The Gazebo driver
  /// \param cf Player config file
  /// \param section Section of the config
  public: IRInterface(player_devaddr_t addr, GazeboDriver *driver,
                              ConfigFile *cf, int section);

  /// \brief Destructor
  public: virtual ~IRInterface();

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

  /// \brief The gazebo laser interface
  private: IRIface *iface;

  /// \brief Gazebo id. This needs to match and ID in a Gazebo WorldFile
  private: char *gz_id;

  /// \brief Timestamp on last data update
  private: double datatime;


  private: player_ir_data_t data;
  private: static boost::recursive_mutex *mutex;
};
/// \}
/// \}
}
#endif
