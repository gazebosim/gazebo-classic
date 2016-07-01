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
/* Desc: Camera Interface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 */

#ifndef CAMERAINTERFACE_HH
#define CAMERAINTERFACE_HH

#include <string>
#include "gazebo/util/system.hh"
#include "GazeboInterface.hh"

/// \addtogroup player_iface
/// \{
/// \defgroup camera_player Camera interface
/// \brief Camera interface
/// \{
/// \brief Camera interface
class GAZEBO_VISIBLE CameraInterface : public GazeboInterface
{
  /// \brief Constructor
  public: CameraInterface(player_devaddr_t _addr, GazeboDriver *_driver,
                          ConfigFile *_cf, int _section);

  /// \brief Destructor
  public: virtual ~CameraInterface();

  /// \brief Handle all messages. This is called from GazeboDriver
  public: virtual int ProcessMessage(QueuePointer &_respQueue,
                                     player_msghdr_t *_hdr, void *_data);

  /// \brief Update this interface, publish new info.
  public: virtual void Update();

  /// \brief Open a SHM interface when a subscription is received.
  ///        This is called fromGazeboDriver::Subscribe
  public: virtual void Subscribe();

  /// \brief Close a SHM interface. This is called from
  ///        GazeboDriver::Unsubscribe
  public: virtual void Unsubscribe();

  private: void OnImage(ConstImageStampedPtr &_msg);

  /// Save a camera frame
  /// This function is commented out because it's not used.
  /// private: void SaveFrame(const char *filename) const;

  /// \brief Gazebo id. This needs to match and ID in a Gazebo WorldFile
  private: std::string cameraName;

  /// \brief Timestamp on last data update
  private: double datatime;

  /// Most recent data
  private: player_camera_data_t data;

  private: gazebo::transport::SubscriberPtr cameraSub;

  // Save image frames?
  private: int save;
  private: int frameno;
};
/// \}
/// \}
#endif
