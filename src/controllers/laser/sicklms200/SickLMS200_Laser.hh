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
 * Desc: Sick LMS 200 laser controller.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN: $Id$
 */

#ifndef SICKLMS200_LASER_HH
#define SICKLMS200_LASER_HH

#include "Controller.hh"

namespace libgazebo
{
  class LaserIface;
  class FiducialIface;
}

namespace gazebo
{
  class RaySensor;

/// @addtogroup gazebo_controller
/// @{
/** \defgroup sicklms200 sicklms200

  \brief Sick LMS 200 laser controller.
   
   This is a controller that collects data from a ray sensor, and populates a libgazebo laser interface. 

  \verbatim
  <model:physical name="laser_model">
    <body:box name="laser_body">

      <sensor:ray name="laser">
        <controller:sicklms200_laser name="controller-name">
          <interface:laser name="iface-name"/>
        </controller:sicklms200_laser>
      </sensor:ray>

    </body:box>
  </model:physical>
  \endverbatim
 
\{
*/

/// \brief Sick LMS 200 laser controller.
/// 
/// This is a controller that simulates a Sick LMS 200
class SickLMS200_Laser : public Controller
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: SickLMS200_Laser(Entity *parent);

  /// \brief Destructor
  public: virtual ~SickLMS200_Laser();

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

  /// \brief Put laser data to the iface
  private: void PutLaserData();

  /// \brief Put fiducial data to the iface
  private: void PutFiducialData();

  /// The laser interface
  private: libgazebo::LaserIface *laserIface;

  private: libgazebo::FiducialIface *fiducialIface;

  /// The parent sensor
  private: RaySensor *myParent;

};

/** /} */
/// @}

}

#endif

