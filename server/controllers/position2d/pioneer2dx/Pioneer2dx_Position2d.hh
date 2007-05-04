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
 * Desc: Position2d controller for a Pioneer2dx.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN: $Id$
 */

#ifndef PIONEER2DX_POSITION2D_HH
#define PIONEER2DX_POSITION2D_HH

#include "Controller.hh"

namespace gazebo
{

/// @addtogroup controllers
/// @{

/// Pioneer 2 DX Position2D controller.
/// This is a controller that simulates a Pioneer 2DX motion
class Pioneer2dx_Position2d : public Controller
{
  /// Constructor
  public: Pioneer2dx_Position2d();

  /// Destructor
  public: virtual ~Pioneer2dx_Position2d();

  /// Load the controller
  /// \param node XML config node
  /// \return 0 on success
  public: virtual int LoadChild(XMLConfigNode *node);

  /// Init the controller
  /// \return 0 on success
  public: virtual int InitChild();

  /// Update the controller
  /// \return 0 on success
  public: virtual int UpdateChild();

  /// Finalize the controller
  /// \return 0 on success
  public: virtual int FiniChild();

};

/// @}

}

#endif
