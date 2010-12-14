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
#include <vector>

#include "Time.hh"
#include "SingletonT.hh"

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
  class SimulationApp;
  class Server;
  class XMLConfig;
  class XMLConfigNode;
  class GazeboConfig;
  class OgreAdaptor;
  class Entity;
  class Common;
  class Body;
  class Model;
  class World;
  class Plugin;
  class Message;

  /// \brief The World
  /*
   * Top level class: Takes care of World Gui SimulatorIface and Server
   * 
   */
  class Simulator : public SingletonT<Simulator>
  {
      /// \brief Private constructor
      private: Simulator();
  
      /// \brief Private destructor
      private: ~Simulator();
  
      /// \brief Closes the present simulation, frees the resources 
      public: void Close();
  
      /// \brief Load the world configuration file 
      public: void Load(const std::string &fileName);
 
      /// \brief Save the world configuration file
      public: void Save(const std::string& filename=std::string());
  
      /// \brief Initialize the simulation
      public: void Init( );
  
      /// \brief Finalize the simulation
      public: void Fini( );

      /// \brief Create a world
      public: World *CreateWorld( const std::string &filname );

      /// \brief Remove a world
      public: void RemoveWorld( const std::string &worldname );

      /// \brief Get the number of world
      public: unsigned int GetWorldCount() const;

      /// \brief Get a world by number
      public: World *GetWorld(unsigned int i) const;

      /// \brief Set the active world
      public: void SetActiveWorld(unsigned int i);

      /// \brief Set the active world
      public: void SetActiveWorld(World *world);

      /// \brief Get the currently active world
      public: World *GetActiveWorld() const;


      /// \brief Main simulation loop, when this loop ends the simulation finish
      public: void Run();
  
      public: void GraphicsUpdate();
  
      /// \brief Gets the local configuration for this computer
      public: GazeboConfig *GetGazeboConfig() const;
  
      /// \brief Gets our current GUI interface
      public: OgreAdaptor *GetRenderEngine() const;
  
      //User Iteractions
      /// \brief Simulator finished by the user
      public: void SetUserQuit();
  
      /// \brief True if the gui is to be used
      public: void SetGuiEnabled( bool enabled );
  
      /// \brief Return true if the gui is enabled
      public: bool GetGuiEnabled() const;
  
      /// \brief True if the gui is to be used
      public: void SetRenderEngineEnabled( bool enabled );
  
      /// \brief Return true if the gui is enabled
      public: inline bool GetRenderEngineEnabled() const
              { return this->renderEngineEnabled; }
  
      /// \brief Set the length of time the simulation should run.
      public: void SetTimeout(double time);
  
      /// \brief Set the physics enabled/disabled
      public: void SetPhysicsEnabled(bool enabled);
  
      /// \brief Get the physics enabled/disabled
      public: inline bool GetPhysicsEnabled() const 
              {return this->physicsEnabled;}
  
      /// \brief Get the number of plugins
      public: unsigned int GetPluginCount() const;
  
      /// \brief Get the name of a plugin
      public: std::string GetPluginName(unsigned int i) const;
  
      /// \brief Add a plugin
      public: void AddPlugin(const std::string &plugin, const std::string &handle);
  
      /// \brief Remove a plugin
      public: void RemovePlugin(const std::string &plugin);

      /// \brief Send a message
      public: void SendMessage( const Message &message );
  
      /// \brief Function to run gui. Used by guiThread
      private: void PhysicsLoop();
  
      ///pointer to the XML Data
      private: XMLConfig *xmlFile;
  
      /// Pointer to the selected Gui 
      private: SimulationApp *gui;
  
      private: OgreAdaptor *renderEngine;
  
      private: GazeboConfig *gazeboConfig;
  
      //upper limits on updating
      //how many updates we have done in this slot
      private: int physicsUpdates;
  
      // render updates
      private: int  renderUpdates;
  
      /// Set to true to increment the simulation once. This is only
      ///  valid when paused.
      private: bool stepInc;
  
      //The user has somewhat signaled the end of the program
      private: bool userQuit;
      private: bool physicsQuit;
  
      /// True if the GUI is enabled
      private: bool guiEnabled;
  
      /// True if the Rendering Engine is enabled
      private: bool renderEngineEnabled;
  
      /// True if physics is enabled
      private: bool physicsEnabled;
  
      private: static std::string defaultWorldXML;
      private: static std::string defaultConfigXML;
      private: std::vector< World* > worlds;
      private: unsigned int activeWorldIndex;
  
      private: std::vector<Plugin*> plugins;

      //Singleton implementation
      private: friend class DestroyerT<Simulator>;
      private: friend class SingletonT<Simulator>;
  };
  
  
/// \}
}

#endif
