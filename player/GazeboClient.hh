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
/* Desc: Gazebo (simulator) client functions 
 * Author: Nate Koenig, Andrew Howard
 * Date: 2 March 2006
 * CVS: $Id: GazeboClient.hh,v 1.1.2.1 2006/12/16 22:43:22 natepak Exp $
 */

#ifndef GAZEBOCLIENT_HH
#define GAZEBOCLIENT_HH

#include "gazebo.h"

/// @brief Gazebo client handler
///
/// This class handles the Gazebo client object, and acts as a shared
/// data-structure for all Gazebo related drivers.  Note that there
/// can only be one instance of this class (it is entirely static).
class GazeboClient
{
  /// Initialize 
  public: static int Init(int serverid, const char *prefixid);

  /// Finalize
  public: static int Fini();

  /// The prefix used for all gazebo ID's
  public: static const char *prefixId;

  /// The one and only gazebo client
  public: static Client *client;

  public: static SimulationIface *sim;
};

#endif
