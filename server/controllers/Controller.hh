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
 * Desc: Controller base class.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN: $Id$
 */
#ifndef CONTROLLER_HH
#define CONTROLLER_HH

#include <string>
#include <vector>

#include "Param.hh"
#include "gz.h"

namespace libgazebo
{
  class Iface;
}

namespace gazebo
{
  class XMLConfigNode;
  class Entity;

/// \addtogroup gazebo_controller
/// \brief Base class for all controllers
/// \{

/// \brief Base class for all controllers
class Controller
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: Controller( Entity *parent );

  /// \brief Destructor
  public: virtual ~Controller();

  /// \brief Load the controller. Called once on startup
  /// \param node The XMLConfig node pointer
  public: void Load(XMLConfigNode *node);

  /// \brief Save the controller in XML format.
  /// \param stream The output stream
  public: void Save(std::string &prefix, std::ostream &stream);

  /// \brief Initialize the controller. Called once on startup.
  /// \return 0 on success
  public: void Init();

  /// \brief Reset the controller
  public: void Reset();

  /// \brief Update the controller. Called every cycle.
  /// \param params Parameters to the update cycle
  public: void Update();

  /// \brief Finialize the controller. Called once on completion.
  public: void Fini();

  /// \brief Return true if an interface is open 
  public: bool IsConnected() const;

  /// \brief Load function for the child class
  protected: virtual void LoadChild(XMLConfigNode * /*node*/) {return;}

  /// \brief Save function for the child class
  protected: virtual void SaveChild(std::string &prefix, std::ostream &stream) {return;}

  /// \brief Init function for the child class
  protected: virtual void InitChild() {return;}

  /// \brief Reset function for the child class
  protected: virtual void ResetChild() {return;}

  /// \brief Update function for the child class
  protected: virtual void UpdateChild() {return;}

  /// \brief Fini function for the child class
  protected: virtual void FiniChild() {return;}

  /// \brief Return the name of this controller
  /// \return The name of the controller
  public: std::string GetName() const;
  
  public: void GetInterfaceNames(std::vector<std::string>& list) const;  

  /// \brief Return Iface by type
  /// \param type The type of the iface to retrieve
  /// \param number If several ifaces of the same type present, which one
  /// \return Iface, or exception if not found. 
  protected: libgazebo::Iface* GetIface(std::string type, bool mandatory=true, int number=0);
 
  /// The type of the controller
  protected: ParamT<std::string> *typeP;

  /// \brief The controller's name
  protected: ParamT<std::string> *nameP;

  /// \brief The entity that owns this controller
  protected: Entity *parent;

  /// \brief flag to keep controllers updating continuously
  protected: ParamT<bool> *alwaysOnP;

  /// \brief Update period 
  protected: ParamT<double> *updatePeriodP;

  /// \brief Last update time
  protected: Time lastUpdate;

  /// \brief Array of all the iface for this controller
  private: std::vector<libgazebo::Iface*> ifaces;

  protected: std::vector<Param*> parameters;
};

/// \}

}

#endif

