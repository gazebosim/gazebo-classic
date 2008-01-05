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

#include "SingletonT.hh"

namespace gazebo
{
/// \addtogroup gazebo_server
/// \brief The World
/// \{

// Forward declarations
  class Gui;
  class Server;
  class SimulationIface;
  class XMLConfig;
  class XMLConfigWriter;

/// \brief The World
/*
 * Top level class: Takes care of World Gui SimulatorIface and Server
 * 
 */
class Simulator : public SingletonT<Simulator>
{
  /// Private constructor
  private: Simulator();

  /// Private destructor
  private: ~Simulator();

  public: void Load(const std::string &worldFileName, int serverId );

  public: int Init( );

  public: int Fini( );

  public: void Update();

  public: void MainLoop();

  public: Gui *GetUI();

  public: bool isPaused() const;

  /// Get the simulation time
  /// \return The simulation time
  public: double GetSimTime() const;

  /// Get the pause time
  /// \return The pause time
  public: double GetPauseTime() const;

  /// Get the start time
  /// \return The start time
  public: double GetStartTime() const;

  /// Get the real time (elapsed time)
  /// \return The real time
  public: double GetRealTime() const;

  /// \brief Get the wall clock time
  /// \return The wall clock time
  public: double GetWallTime() const;

  public: void SetUserQuit();

  public: void Save(const std::string& filename);

  public: void LoadGui(XMLConfigNode *rootNode);
  public: void SaveGui(XMLConfigNode *node);
  
  ///pointer to the XML Data
  private:gazebo::XMLConfig *xmlFile;

  /// Pointer to the selected Gui 
  private: Gui *gui;

  private: bool userQuit;

  /// Flag set if simulation is paused
  private: bool pause;

  /// Current simulation time
  private: double simTime, pauseTime, startTime;

  private: friend class DestroyerT<Simulator>;
  private: friend class SingletonT<Simulator>;

};

/// \}
}

#endif
