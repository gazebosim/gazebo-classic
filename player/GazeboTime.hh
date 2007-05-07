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
/* Desc: Gazebo Time functions 
 * Author: Nate Koenig, Andrew Howard
 * Date: 2 March 2006
 * CVS: $Id: GazeboTime.hh,v 1.1.2.1 2006/12/16 22:43:22 natepak Exp $
 */

#ifndef GAZEBOTIME_HH
#define GAZEBOTIME_HH

#include <libplayercore/playercore.h>

namespace gazebo
{

class GazeboTime : public PlayerTime
{
  public: GazeboTime();
  public: virtual ~GazeboTime();

  // Get the simulator time
  public: int GetTime(struct timeval* time);
  public: int GetTimeDouble(double* time);

};

}
#endif
