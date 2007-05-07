/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: Graphics 3d Interface for Player
 * Author: Nate Koenig
 * Date: 30 Jan 2007
 * SVN: $Id$
 */

#ifndef GRAPHICS3DINTERFACE_HH
#define GRAPHICS3DINTERFACE_HH

#include "GazeboInterface.hh"

namespace gazebo
{

// Forward declarations
class Graphics3dIface;


class Graphics3dInterface : public GazeboInterface
{
  /// @brief Constructor
  public: Graphics3dInterface(player_devaddr_t addr, GazeboDriver *driver,
                              ConfigFile *cf, int section);

  /// @brief Destructor
  public: virtual ~Graphics3dInterface();

  /// @brief Handle all messages. This is called from GazeboDriver
  public: virtual int ProcessMessage(MessageQueue *respQueue,
                                     player_msghdr_t *hdr, void *data);

  /// @brief Update this interface, publish new info.
  public: virtual void Update();

  /// @brief Open a SHM interface when a subscription is received. \
  ///        This is called fromGazeboDriver::Subscribe
  public: virtual void Subscribe();

  /// @brief Close a SHM interface. This is called from \
  ///        GazeboDriver::Unsubscribe
  public: virtual void Unsubscribe();

  private: Graphics3dIface *iface;

  /// @brief Gazebo id. This needs to match and ID in a Gazebo WorldFile
  private: char *gz_id;

};

}

#endif
