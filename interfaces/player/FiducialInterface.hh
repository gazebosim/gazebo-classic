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
/* Desc: Fiducial Interface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 */

#ifndef FIDUCIALINTERFACE_HH
#define FIDUCIALINTERFACE_HH

#include "GazeboInterface.hh"

namespace boost
{
  class recursive_mutex;
}

namespace libgazebo
{
/// \addtogroup player_iface
/// \{
/// \defgroup fiduacial_player Fiducial Interface
/// \brief Plugin Player interface for Gazebo fiducials
/// \{
  // Forward declarations
  class FiducialIface;

  /// \brief Plugin Player interface for Gazebo fiducials
  class FiducialInterface : public GazeboInterface
  {
    /// \brief Constructor
    /// \param addr Plaer device address
    /// \param driver The Gazebo driver
    /// \param cf Player config file
    /// \param section Section of the config
    public: FiducialInterface(player_devaddr_t addr, GazeboDriver *driver,
                ConfigFile *cf, int section);

    /// \brief Destructor
    public: virtual ~FiducialInterface();

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

    /// \brief The gazebo fiducial interface
    private: FiducialIface *iface;

    /// \brief Gazebo id. This needs to match and ID in a Gazebo WorldFile
    private: char *gz_id;

    /// \brief Timestamp on last data update
    private: double datatime;

    private: player_fiducial_data_t data;
    private: static boost::recursive_mutex *mutex;
  };
/// \}
/// \}
}
#endif
