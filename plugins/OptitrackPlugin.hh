 /*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#ifndef _OPTITRACK_PLUGIN_HH_
#define _OPTITRACK_PLUGIN_HH_

#include <map>
#include <string>
#include <thread>
#include <vector>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/util/system.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/math/Vector3.hh>

typedef std::map<std::string, std::vector<gazebo::math::Vector3> >
          ModelMarkers;

typedef std::map<std::string, gazebo::math::Pose> ModelPoses;

namespace gazebo
{
  class GAZEBO_VISIBLE OptitrackPlugin : public WorldPlugin
  {
    #define MAX_NAMELENGTH              256

    /// \brief Creates an OptitrackPlugin object able to receive multicast
    /// updates from the Optitrack server containing the tracking information.
    /// \param[in] _server IP address of the optitrack server. This might be
    /// needed for requesting commands for tweaking the tracking behavior. The
    /// server IP is not needed for receiving tracking messages. These
    /// messages are received via multicast.
    /// \param[i] _verbose Whether or not to print incoming packets.
    public: OptitrackPlugin(const bool _verbose = false);

    public: virtual void Load(gazebo::physics::WorldPtr _parent,
      sdf::ElementPtr _sdf);

    public: virtual void Init();

    /// \brief Default destructor.
    public: ~OptitrackPlugin() = default;

    /// \brief Start receiving tracking updates. Each tracking update will be
    /// published as a Gazebo message on topic '~/optitrack'.
    public: void StartReception();

    /// \brief Receive tracking updates and publish them using Gazebo messages.
    private: void RunReceptionTask();

    /// \brief Unpack the data received from the network.
    /// \param[in] _data Buffer received.
    private: void Unpack(char *_data);

    /// \brief Return the status of the OptitrackPlugin client initialization
    /// \return True if OptitrackPlugin data reception is active..
    public: bool IsActive();

    /// ToDo.
    private: bool TimecodeStringify(unsigned int _inTimecode,
                                    unsigned int _inTimecodeSubframe,
                                    char *_buffer,
                                    int _bufferSize);
    /// ToDo.
    private: bool DecodeTimecode(unsigned int _inTimecode,
                                 unsigned int _inTimecodeSubframe,
                                 int *_hour,
                                 int *_minute,
                                 int *_second,
                                 int *_frame,
                                 int *_subframe);

    public: void SetWorld(const std::string &_world);

    /// \brief True if OptitrackPlugin data reception is active
    private: bool active;

    /// \brief OptitrackPlugin multicast address.
    private: std::string MulticastAddress;

    /// \brief Port used for sending/receiving tracking updates.
    private: int PortData;

    /// \brief NatNet major version.
    private: const int NatNetVersionMajor = 2;

    /// \brief NatNet minor version.
    private: const int NatNetVersionMinor = 7;

    /// \brief True if incoming packets will be printed
    private: bool verbose;

    /// \brief UDP socket used to received tracking updates.
    private: int dataSocket;

    /// \brief IP address associated to the multicast socket.
    private: std::string myIPAddress;

    private: std::string world;

    /// \brief Gazebo transport node used to publish tracker poses.
    private: transport::NodePtr gzNode;

    /// \brief Gazebo publishers for each rigid body pose
    private: std::map<std::string, transport::PublisherPtr> trackerPubs;

    /// \brief Names of tracked rigid bodies
    private: std::vector<std::string> trackerNames;

    /// \brief Store names and poses of tracker models to be published
    private: ModelPoses lastModelMap;
  };
}

#endif
