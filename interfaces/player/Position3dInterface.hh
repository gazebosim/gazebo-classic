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
/* Desc: Position 3d Interface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 */

#ifndef POSITION3DINTERFACE_HH
#define POSITION3DINTERFACE_HH

#include "GazeboInterface.hh"

namespace boost
{
  class recursive_mutex;
}

// Forward declarations
namespace libgazebo
{
/// \addtogroup player_iface Interfaces
/// \{
/// \defgroup position3d_player Position3d Interface
/// \brief Position3d interface
/// \{

  class PositionIface;

  /// \brief Position3d interface
  class Position3dInterface : public GazeboInterface
  {
    /// \brief Constructor
    public: Position3dInterface(player_devaddr_t addr, GazeboDriver *driver,
                ConfigFile *cf, int section);

    /// \brief Destructor
    public: virtual ~Position3dInterface();

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

    private: PositionIface *iface;

    /// \brief Gazebo id. This needs to match and ID in a Gazebo WorldFile
    private: char *gz_id;

    /// \brief Timestamp on last data update
    private: double datatime;
    private: static boost::recursive_mutex *mutex;
  };
}

/// \}
/// \}

#endif


