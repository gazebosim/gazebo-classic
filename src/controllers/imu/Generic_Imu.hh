/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

