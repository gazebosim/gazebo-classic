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
/*
 * Desc: generic IMU controller.
 * Author: Matt Thompson
 * Date: 07 Sep 2008
 * SVN: $Id$
 */

#ifndef GENERIC_IMU_HH
#define GENERIC_IMU_HH

#include "Controller.hh"

namespace libgazebo
{
  class ImuIface;
}

namespace gazebo
{
  class ImuSensor;

class Generic_Imu : public Controller
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: Generic_Imu(Entity *parent);

  /// \brief Destructor
  public: virtual ~Generic_Imu();

  /// \brief Load the controller
  /// \param node XML config node
  /// \return 0 on success
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// \brief Init the controller
  /// \return 0 on success
  protected: virtual void InitChild();

  /// \brief Update the controller
  /// \return 0 on success
  protected: virtual void UpdateChild();

  /// \brief Finalize the controller
  /// \return 0 on success
  protected: virtual void FiniChild();

  /// \brief Put IMU data to the iface
  private: void PutImuData();

  /// The IMU interface
  private: libgazebo::ImuIface *imuIface;

  /// The parent sensor
  private: ImuSensor *myParent;

};

/** /} */
/// @}

}

#endif

