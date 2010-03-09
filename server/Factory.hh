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
 * Desc: Factory controller.
 * Author: Nathan Koenig
 * Date: 29 July 2007
 * SVN: $Id: Factory.hh 7551 2009-03-27 16:15:13Z natepak $
 */
#ifndef FACTORY_HH
#define FACTORY_HH

#include "Controller.hh"
#include "Entity.hh"
#include "gz.h"


namespace gazebo
{
  class FactoryIface;

/// \addtogroup gazebo_server
/// \{
/** \defgroup factory factory
  \brief Factory used for dynamic construction of models

  The factory controller allows dynamic addition and deletion of models using libgazebo's \ref factory_iface interface.

\{
*/

/// \brief Factory used for dynamic construction of models
class Factory
{
  /// Constructor
  public: Factory();

  /// Destructor
  public: virtual ~Factory();

  /// Init the controller
  /// \return 0 on success
  public: void Init();

  /// Update the controller
  /// \return 0 on success
  public: void Update();

  /// The Position interface
  private: FactoryIface *factoryIface;

  private: std::string xmlPrefix;
  private: std::string xmlSuffix;
};

/** \} */
/// \}

}

#endif

