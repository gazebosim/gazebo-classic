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
#ifndef _SERVER_HH_
#define _SERVER_HH_

#include <string>
#include <list>

#include <sdf/sdf.hh>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/util/system.hh"


namespace gazebo
{
  class Master;

  // forward declaration of private class
  struct ServerPrivate;

  /// \class Master Master.hh gazebo_core.hh
  /// \brief Base class for simulation server that handles commandline options,
  /// starts a Master, runs World update and sensor generation loops.
  class GAZEBO_VISIBLE Server
  {
    /// \brief Constructor.
    public: Server();

    /// \brief Destructor.
    public: virtual ~Server();

    /// \brief Output help about gzserver.
    public: void PrintUsage();

    /// \brief Parse command line arguments.
    /// \param[in] _argc Number of arguments.
    /// \param[in] _argv Array of argument values.
    /// \return True on success.
    public: bool ParseArgs(int _argc, char **_argv);

    /// \brief Preload the server.
    /// \return True if load was successful.
    public: bool PreLoad();

    /// \brief Load a world file and optionally override physics engine type.
    /// \param[in] _filename Name of the world file to load.
    /// \param[in] _physics Physics engine type (ode|bullet|dart|simbody).
    /// \return True on success.
    public: bool LoadFile(const std::string &_filename="worlds/empty.world",
                          const std::string &_physics="");

    /// \brief Load the Server from an SDF string.
    /// \param[in] _sdfString SDF string from which to load a World.
    /// \return True on success.
    public: bool LoadString(const std::string &_sdfString);

    /// \brief Run the Server.
    public: void Run();

    /// \brief Stop the Server.
    public: void Stop();

    /// \brief Finalize the Server.
    public: void Fini();

    /// \brief Set the parameters.
    /// \param[in] _params Map of string parameters
    public: void SetParams(const common::StrStr_M &_params);

    /// \brief Get whether the Server has been initialized.
    /// \return True if initialized.
    public: bool GetInitialized() const;

    /// \brief Load implementation.
    /// \param[in] _elem Description of the world to load.
    /// \param[in] _physics Physics engine type (ode|bullet|dart|simbody).
    private: bool LoadImpl(sdf::ElementPtr _elem,
                           const std::string &_physics="");

    /// \brief SIGINT handler
    /// \param[in] _v Unused.
    private: static void SigInt(int _v);

    /// \brief Process all command line parameters.
    private: void ProcessParams();

    /// \brief Receive a control message, and push it onto a queue.
    /// \param[in] _msg Message that is received.
    private: void OnControl(ConstServerControlPtr &_msg);

    /// \brief Open a new world.
    /// \param[in] _filename Name and path of the world to open.
    /// \return True on success.
    private: bool OpenWorld(const std::string &_filename);

    /// \brief Handle all control messages.
    private: void ProcessControlMsgs();

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<ServerPrivate> dataPtr;
  };
}

#endif
