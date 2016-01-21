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
/* Desc: Gazebo Driver for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 */

#ifndef GAZEBO_PLAYER_DRIVER_HH
#define GAZEBO_PLAYER_DRIVER_HH

#include <unistd.h>
#include <string.h>
#include <vector>

#include "player.h"
#include "gazebo/util/system.hh"

extern "C"
{
  GAZEBO_VISIBLE int player_driver_init(DriverTable *_table);
}

/// \addtogroup player
/// \brief Gazebo player driver
/// \{
// Forward declarations
class GazeboInterface;

/// \brief Gazebo player driver
class GAZEBO_VISIBLE GazeboDriver : public Driver
{
  /// \brief Constructor
  public: GazeboDriver(ConfigFile *_cf, int _section);

  /// \brief Destructor
  public: virtual ~GazeboDriver();

  /// \brief Set up the device.  Return 0 if things go well, and -1 otherwise.
  public: virtual int Setup();

  /// \brief Shutdown the device
  public: virtual int Shutdown();

  /// \brief Process all messages for this driver.
  public: virtual int ProcessMessage(QueuePointer &respQueue,
                                     player_msghdr * hdr,
                                     void * data);

  /// \brief Subscribe an device to this driver
  public: virtual int Subscribe(player_devaddr_t addr);

  /// \brief Remove a device from this driver
  public: virtual int Unsubscribe(player_devaddr_t addr);

  /// \brief The server thread calls this method frequently.
  ///        We use it to check  for new commands and configs
  private: virtual void Update();

  /// \brief Helper function to load all devices on startup
  private: int LoadDevices(ConfigFile* cf, int section);

  /// \brief Find a device according to a player_devaddr
  private: GazeboInterface *LookupDevice(player_devaddr_t addr);

  /// Array of interfaces associated with this driver
  protected: std::vector<GazeboInterface *> devices;

  /// \brief Max device count
  protected: int deviceMaxCount;
};

/// \}

#endif


