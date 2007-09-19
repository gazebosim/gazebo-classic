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

#ifndef CONTACTPARAMS_HH
#define CONTACTPARAMS_HH

namespace gazebo
{

class ContactParams
{
  // Constructor
  public: ContactParams();

  // Spring constant
  public: double kp;   

  // Damping constant
  public: double kd;

  // 0..1, 0=no bounciness
  public: double bounce;
  
  // coefficients of friction 
  public: double mu1,mu2;

  // Force-dependent-slip direction 1 and 2
  public: double slip1,slip2;   

  public: double bounceVel;

  public: double softCfm;
};

}
#endif
