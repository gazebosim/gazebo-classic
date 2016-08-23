/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_GZ_HH_
#define _GAZEBO_GZ_HH_

#include <string>
#include <list>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>

#include "gazebo/transport/transport.hh"
#include "gazebo/common/common.hh"
#include "gazebo/msgs/msgs.hh"

namespace gazebo
{
  namespace po = boost::program_options;

  /// \brief Base class for all commands
  class Command
  {
    /// \brief Constructor
    /// \param[in] _name Name of the command.
    /// \param[in] _brief One line command description.
    public: Command(const std::string &_name, const std::string &_brief);

    /// \brief Destructor
    public: virtual ~Command();

    /// \brief Print help information.
    public: void Help();

    /// \brief Print detailed help.
    public: virtual void HelpDetailed() = 0;

    /// \brief Get the one description.
    /// \return The brief description of the command.
    public: std::string GetBrief() const;

    /// \brief Execute the command.
    /// \param[in] _argc Number of command line arguments.
    /// \param[in] _argv The line arguments.
    public: bool Run(int _argc, char **_argv);

    /// \brief Process signal interrupt.
    public: static void Signal();

    /// \brief List all the command options.
    public: void ListOptions();

    /// \brief Return true if transport is need for the command.
    /// \return True if transport should be initialized.
    protected: virtual bool TransportRequired();

    /// \brief Implementation of Run
    /// \return True on success
    protected: virtual bool RunImpl() = 0;

    /// \brief Initialize transport.
    /// \return True on success
    protected: bool TransportInit();

    /// \brief Finalized transport.
    /// \return True on success
    protected: bool TransportFini();

    /// \brief Name of the command.
    protected: std::string name;

    /// \brief One line description of the command.
    protected: std::string brief;

    /// \brief Options that are visible to the user
    protected: po::options_description visibleOptions;

    /// \brief Variable map
    protected: po::variables_map vm;

    /// \brief Signal mutex.
    protected: static boost::mutex sigMutex;

    /// \breif Signal condition.
    protected: static boost::condition_variable sigCondition;

    /// \brief Save argc for use by child commands.
    protected: int argc;

    /// \brief Save argv for use by child commands.
    protected: char **argv;
  };

  /// \brief World command
  class WorldCommand : public Command
  {
    /// \brief Constructor
    public: WorldCommand();

    // Documentation inherited
    public: virtual void HelpDetailed();

    // Documentation inherited
    protected: virtual bool RunImpl();
  };

  /// \brief Physics command
  class PhysicsCommand : public Command
  {
    /// \brief Constructor
    public: PhysicsCommand();

    // Documentation inherited
    public: virtual void HelpDetailed();

    // Documentation inherited
    protected: virtual bool RunImpl();
  };

  /// \brief Model command
  class ModelCommand : public Command
  {
    /// \brief Constructor
    public: ModelCommand();

    // Documentation inherited
    public: virtual void HelpDetailed();

    // Documentation inherited
    protected: virtual bool RunImpl();

    /// \brief Spawn helper function.
    /// \param[in] _sdf SDF model to spawn.
    /// \param[in] _name Name for the model.
    /// \param[in] _pose Pose of the model.
    /// \param[in] _node Node for communication.
    /// \return True if the spawn message was sent.
    private: bool ProcessSpawn(sdf::SDFPtr _sdf,
                 const std::string &_name, const math::Pose &_pose,
                 transport::NodePtr _node);
  };

  /// \brief Joint command
  class JointCommand : public Command
  {
    /// \brief Constructor
    public: JointCommand();

    // Documentation inherited
    public: virtual void HelpDetailed();

    // Documentation inherited
    protected: virtual bool RunImpl();
  };

  /// \brief Camera command
  class CameraCommand : public Command
  {
    /// \brief Constructor
    public: CameraCommand();

    // Documentation inherited
    public: virtual void HelpDetailed();

    // Documentation inherited
    protected: virtual bool RunImpl();
  };

  /// \brief Stats command
  class StatsCommand : public Command
  {
    /// \brief Constructor
    public: StatsCommand();

    // Documentation inherited
    public: virtual void HelpDetailed();

    // Documentation inherited
    protected: virtual bool RunImpl();

    /// \brief World statistics callback.
    /// \param[in] _msg World statistics message.
    private: void CB(ConstWorldStatisticsPtr &_msg);

    /// \brief Sim time buffer
    private: std::list<common::Time> simTimes;

    /// \brief Real time buffer
    private: std::list<common::Time> realTimes;
  };

  /// \brief SDF command
  class SDFCommand : public Command
  {
    /// \brief Constructor
    public: SDFCommand();

    // Documentation inherited
    public: virtual void HelpDetailed();

    // Documentation inherited
    protected: virtual bool RunImpl();

    // Documentation inherited
    protected: virtual bool TransportRequired();
  };

  /// \brief Help command
  class HelpCommand : public Command
  {
    /// \brief Constructor
    public: HelpCommand();

    // Documentation inherited
    public: virtual void HelpDetailed();

    // Documentation inherited
    protected: virtual bool RunImpl();

    // Documentation inherited
    protected: virtual bool TransportRequired();

    /// \brief Displays help message for specified command.
    /// \param[in] _command Command to display help message.
    private: void Help(const std::string &_command);
  };

  /// \brief Debug command
  class DebugCommand : public Command
  {
    /// \brief Constructor
    public: DebugCommand();

    // Documentation inherited
    public: virtual void HelpDetailed();

    // Documentation inherited
    protected: virtual bool RunImpl();

    // Documentation inherited
    protected: virtual bool TransportRequired();
  };
}
#endif
