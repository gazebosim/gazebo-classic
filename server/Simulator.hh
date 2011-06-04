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
/* Desc: The Simulator; Top level managing object
 * Author: Jordi Polo
 * Date: 3 Jan 2008
 */

#ifndef SIMULATOR_HH
#define SIMULATOR_HH

#include <string>
#include <boost/thread.hpp>
#include <boost/signal.hpp>

#include "Time.hh"
#include "SingletonT.hh"
#include "Timer.hh"

namespace boost
{
  class recursive_mutex;
}

namespace gazebo
{
/// \addtogroup gazebo_server
/// \brief The World
/// \{

// Forward declarations
  class Gui;
  class Server;
  class XMLConfig;
  class XMLConfigNode;
  class GazeboConfig;
  class OgreAdaptor;
  class Entity;
  class Common;
  class Body;
  class Model;
  class Plugin;

/// \brief The World
/*
 * Top level class: Takes care of World Gui SimulatorIface and Server
 * 
 */
  class Simulator : public SingletonT<Simulator>
  {
    public: enum State {LOAD, INIT, RUN};

    /// \brief Private constructor
    private: Simulator();

    /// \brief Private destructor
    private: ~Simulator();

    /// \brief Closes the present simulation, frees the resources 
    public: void Close();

    /// \brief Load the world configuration file 
    // DEPRECATED
    public: void Load(const std::string &worldFileName, int serverId ) __attribute__ ((deprecated));

    public: void LoadWorldFile(const std::string &worldFileName, int serverId);

    public: void LoadWorldString(const std::string &worldString, int serverId);

    /// \brief Load the world configuration file
    public: void Load(gazebo::XMLConfig* worldConfig, int serverId);

    /// \brief Save the world configuration file
    public: void Save(const std::string& filename=std::string());

    /// \brief Initialize the simulation
    public: void Init( );

    /// \brief Finalize the simulation
    public: void Fini( );

    /// \brief Main simulation loop, when this loop ends the simulation finish
    public: void MainLoop();

    /// \brief Gets the local configuration for this computer
    public: GazeboConfig *GetGazeboConfig() const;

    /// \brief Gets our current GUI interface
    public: OgreAdaptor *GetRenderEngine() const;

    /// \brief Returns the state of the simulation true if paused
    public: bool IsPaused() const;

    /// \brief Set whether the simulation is paused
    public: void SetPaused(bool p);

    /// Get the simulation time
    /// \return The simulation time
    public: Time GetSimTime() const;

    /// \brief Set the sim time
    public: void SetSimTime(Time t);

    /// Get the pause time
    /// \return The pause time
    public: Time GetPauseTime() const;

    /// Get the start time
    /// \return The start time
    public: Time GetStartTime() const;

    /// Get the real time (elapsed time)
    /// \return The real time
    public: inline Time GetRealTime() const
            {
              return Time::GetWallTime() - this->startTime;
            }

    /// Get the real time (elapsed time)
    /// \return The real time
    public: inline Time GetWallTime() const
            {
              std::cout << "gazebo::Simulator::GetWallTime() is deprecated, please use GetRealTime() instead\n";
              return this->GetRealTime();
            }

    //User Iteractions
    /// \brief Simulator finished by the user
    public: void SetUserQuit();

    /// \brief Return true if the step has incremented
    public: bool GetStepInc() const;

    /// \brief Set whether the step has incremented
    public: void SetStepInc(bool step);

    /// \brief True if the gui is to be used
    public: void SetGuiEnabled( bool enabled );

    /// \brief Return true if the gui is enabled
    public: bool GetGuiEnabled() const;

    /// \brief True if the gui is to be used
    public: void SetRenderEngineEnabled( bool enabled );

    /// \brief Return true if the gui is enabled
    public: bool GetRenderEngineEnabled() const;

    /// \brief Set the length of time the simulation should run.
    public: void SetTimeout(double time);

    /// \brief Set the physics enabled/disabled
    public: void SetPhysicsEnabled(bool enabled);

    /// \brief Get the physics enabled/disabled
    public: bool GetPhysicsEnabled() const;

    /// \brief Get the model that contains the entity
    public: Model *GetParentModel( Entity *entity ) const;

    /// \brief Get the body that contains the entity
    public: Body *GetParentBody( Entity *entity ) const;

    /// \brief Get the simulator mutex for rendering lock
    public:boost::recursive_mutex *GetMRMutex();

    /// \brief Get the simulator mutex for model deletion lock
    public:boost::recursive_mutex *GetMDMutex();

    /// \brief Get the state of the simulation
    public: State GetState() const;

    /// \brief Get the number of plugins
    public: unsigned int GetPluginCount() const;

    /// \brief Get the name of a plugin
    public: std::string GetPluginName(unsigned int i) const;

    /// \brief Add a plugin
    public: void AddPlugin(const std::string &plugin, const std::string &handle);

    /// \brief Set whether a Ogre.log file should be created
    public: void SetCreateOgreLog(bool v);

    /// \brief Return true if a Ogre.log file should be created
    public: bool GetCreateOgreLog() const;

    /// \brief Remove a plugin
    public: void RemovePlugin(const std::string &plugin);

    /// \brief Connect a boost::slot the the pause signal
    public: template<typename T>
            void ConnectPauseSignal( T subscriber )
            {
              pauseSignal.connect(subscriber);
            }

     /// \brief Connect a boost::slot the the step signal
    public: template<typename T>
            void ConnectStepSignal( T subscriber )
            {
              stepSignal.connect(subscriber);
            }
 
    /// \brief Function to run gui. Used by guiThread
    private: void PhysicsLoop();

    ///pointer to the XML Data
    private: XMLConfig *xmlFile;

    /// Pointer to the selected Gui 
    private: Gui *gui;

    private: OgreAdaptor *renderEngine;

    private: GazeboConfig *gazeboConfig;

    /// Flag to know if we have a simulation loaded
    private: bool loaded;

    /// Flag set if simulation is paused
    private: bool pause;

    /// Current simulation time
    private: Time simTime, pauseTime, startTime;

    //upper limits on updating
    //how many updates we have done in this slot
    private: int physicsUpdates;

    // when the slot started
    private: double checkpoint;

    // render updates
    private: int  renderUpdates;

    /// Set to true to increment the simulation once. This is only
    ///  valid when paused.
    private: bool stepInc;

    //The user has somewhat signaled the end of the program
    private: bool userQuit;
    private: bool renderQuit;

    /// True if the GUI is enabled
    private: bool guiEnabled;

    /// True if the Rendering Engine is enabled
    private: bool renderEngineEnabled;

    /// True if physics is enabled
    private: bool physicsEnabled;

    private: bool ogreLog;

    /// Length of time the simulation should run
    private: double timeout;

    /// Length of time the simulator is allowed to run
    private: double updateTime;

    /// Thread in which to run the gui
    private: boost::thread *physicsThread;

    private: boost::recursive_mutex *render_mutex;
    private: boost::recursive_mutex *model_delete_mutex;

    private: State state;

    private: boost::signal<void (bool)> pauseSignal;
    private: boost::signal<void (bool)> stepSignal;

    //Singleton implementation
    private: friend class DestroyerT<Simulator>;
    private: friend class SingletonT<Simulator>;

    private: static std::string defaultWorld;

    private: std::vector<Plugin*> plugins;
};


/// \}
}

#endif
