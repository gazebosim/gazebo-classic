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
/* Desc: Specification of a contact
 * Author: Nate Koenig
 * Date: 10 Nov 2009
 * SVN: $Id$
 */

#ifndef JOINTFEEDBACK_HH
#define JOINTFEEDBACK_HH

#include "Vector3.hh"

namespace gazebo
{
  class JointFeedback
  {
    /// \brief Operator =
    public: const JointFeedback &operator=(const JointFeedback &f)
            {
              this->body1Force = f.body1Force;
              this->body2Force = f.body2Force;

              this->body1Torque = f.body1Torque;
              this->body2Torque = f.body2Torque;
              return *this;
            }

    public: Vector3 body1Force;
    public: Vector3 body2Force;
  
    public: Vector3 body1Torque;
    public: Vector3 body2Torque;
  };
}

#endif
