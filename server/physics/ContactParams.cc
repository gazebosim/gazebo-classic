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
/* Desc: Parameters for contact joints
 * Author: Nate Koenig
 * Date: 30 July 2003
 * CVS: $Id$
 */

#include <ode/ode.h>
#include "ContactParams.hh"

//////////////////////////////////////////////////////////////////////////////
// Default constructor
ContactParams::ContactParams()
{
  kp = 10000.0; //dInfinity;
  kd = 0;
  bounce = 0.0;
  
  mu1 = dInfinity;
  mu2 = dInfinity;
  slip1 = 0.01;
  slip2 = 0.01;
}
