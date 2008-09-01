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
 * Desc: A stereo camera controller
 * Author: Nathan Koenig
 * Date: 06 April 2008
 * SVN: $Id:$
 */

#ifndef STEREO_CAMERA_HH
#define STEREO_CAMERA_HH

#include <map>

#include "Stereo_Camera.hh"
#include "Param.hh"
#include "Controller.hh"

namespace gazebo
{
  class CameraIface;
  class StereoCameraIface;

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
  private: void PutCameraData(CameraData *camera_data, unsigned int camera);

  /// The camera interface
  private: StereoCameraIface *stereoIface;
  private: std::map< std::string, CameraIface*> cameraIfaces;

  private: ParamT<std::string> *leftCameraNameP;
  private: ParamT<std::string> *rightCameraNameP;

  /// The parent sensor
  private: StereoCameraSensor *myParent;

};

/** /} */
/// @}

}

#endif

