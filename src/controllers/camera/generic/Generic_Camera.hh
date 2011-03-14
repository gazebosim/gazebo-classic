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
 * Desc: A generic camera controller
 * Author: Nathan Koenig
 * Date: 07 July 2007
 * SVN: $Id$
 */

#ifndef GENERIC_CAMERA_HH
#define GENERIC_CAMERA_HH

#include "Controller.hh"

namespace libgazebo
{
  class CameraIface;
}

namespace gazebo
{
  class MonoCameraSensor;

/// @addtogroup gazebo_controller
/// @{
/** \defgroup genericcamera generic camera

  \brief Generic camera controller.
  
  This is a controller that collects data from a Camera Sensor and populates a libgazebo camera interfaace. This controller should only be used as a child of a camera sensor 

  \verbatim
  <model:physical name="camera_model">
    <body:empty name="camera_body">
      <sensor:camera name="camera_sensor">

        <controller:generic_camera name="controller-name">
          <interface:camera name="iface-name"/>
        </controller:generic_camera>

      </sensor:camera>
    </body:empty>
  </model:phyiscal>
  \endverbatim
 
\{
*/

/// \brief Generic camera controller.
/// 
/// This is a controller that simulates a generic camera
class Generic_Camera : public Controller
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: Generic_Camera(Entity *parent);

  /// \brief Destructor
  public: virtual ~Generic_Camera();

  /// \brief Load the controller
  /// \param node XML config node
  /// \return 0 on success
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// \brief Save the controller.
  /// \stream Output stream
  protected: void SaveChild(std::string &prefix, std::ostream &stream);

  /// \brief Init the controller
  /// \return 0 on success
  protected: virtual void InitChild();

  /// \brief Update the controller
  /// \return 0 on success
  protected: virtual void UpdateChild();

  /// \brief Finalize the controller
  /// \return 0 on success
  protected: virtual void FiniChild();

  /// \brief Put camera data to the iface
  private: void PutCameraData();

  /// The camera interface
  private: libgazebo::CameraIface *cameraIface;

  /// The parent sensor
  private: MonoCameraSensor *myParent;

};

/** /} */
/// @}

}

#endif

