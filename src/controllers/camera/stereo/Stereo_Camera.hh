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
 * Desc: A stereo camera controller
 * Author: Nathan Koenig
 * Date: 06 April 2008
 * SVN: $Id$
 */

#ifndef STEREO_CAMERA_HH
#define STEREO_CAMERA_HH

#include <map>

#include "Stereo_Camera.hh"
#include "common/Param.hh"
#include "Controller.hh"

namespace libgazebo
{
  class CameraIface;
  class StereoCameraIface;
}


namespace gazebo
{
/// @addtogroup gazebo_controller
/// @{
/** \defgroup stereocamera stereo camera

  \brief Stereo camera controller.
  
  This is a controller that collects data from a Stereo Camera Sensor and populates a libgazebo stereo camera interfaace. This controller should only be used as a child of a stereo camera sensor 

  \verbatim
  <model:physical name="camera_model">
    <body:empty name="camera_body">
      <sensor:stereocamera name="stereo_camera_sensor">

        <controller:stereo_camera name="controller-name">
          <interface:stereocamera name="iface-name"/>
        </controller:stereo_camera>

      </sensor:stereocamera>
    </body:empty>
  </model:phyiscal>
  \endverbatim
 
\{
*/

/// \brief Stereo camera controller.
/// 
/// This is a controller that simulates a stereo camera
class Stereo_Camera : public Controller
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: Stereo_Camera(Entity *parent);

  /// \brief Destructor
  public: virtual ~Stereo_Camera();

  /// \brief True if a stereo iface is connected
  public: bool StereoIfaceConnected() const;

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

  /// \brief Put stereo data to the iface
  private: void PutStereoData();

  /// \brief Put camera data to the iface
  private: void PutCameraData(libgazebo::CameraData *camera_data, unsigned int camera);

  /// The camera interface
  private: libgazebo::StereoCameraIface *stereoIface;
  private: std::map< std::string, libgazebo::CameraIface*> cameraIfaces;

  private: ParamT<std::string> *leftCameraNameP;
  private: ParamT<std::string> *rightCameraNameP;

  /// The parent sensor
  private: StereoCameraSensor *myParent;

};

/** /} */
/// @}

}

#endif

