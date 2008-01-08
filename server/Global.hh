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
/* Desc: Generic Typedefs, macros, functions, etc
 * Author: Nate Koenig
 * Date: 16 June 2003
 * CVS: $Id$
 */

#ifndef GZGLOBAL_HH
#define GZGLOBAL_HH

/////////////////////////////////////////////////////////////////////////////
// Includes
/////////////////////////////////////////////////////////////////////////////

#include <stdint.h>
#include <list>
#include <math.h>

#include "Pose3d.hh"
#include "GazeboError.hh"
#include "GazeboMessage.hh"


/////////////////////////////////////////////////////////////////////////////
// Defines
/////////////////////////////////////////////////////////////////////////////
#ifndef NULL
#define NULL 0
#endif

// TODO: Fix the version number
#define GAZEBO_VERSION "0.8-pre1"

#ifndef GZ_COLLIDE_BITS

#define GZ_ALL_COLLIDE 0xFFFFFFFF
#define GZ_NONE_COLLIDE 0x00000000
#define GZ_FIXED_COLLIDE 0x00000001
#define GZ_LASER_COLLIDE 0x00000002

#endif

#ifndef GZ_CAMERA_BITS

#define GZ_ALL_CAMERA 0xFFFFFFFF
#define GZ_LASER_CAMERA 0x00000001
#define GZ_GUIDATA_CAMERA 0x00000002

#endif

/////////////////////////////////////////////////////////////////////////////
// Macros
/////////////////////////////////////////////////////////////////////////////

// Convert radians to degrees
#define RTOD(r) ((r) * 180 / M_PI)

// Convert degrees to radians
#define DTOR(d) ((d) * M_PI / 180)

// Normalize an angle in the range -Pi to Pi
#define NORMALIZE(a) (atan2(sin(a), cos(a)))

// Odd and even macros
#define ISEVEN(x) ( ((x) % 2) == 0)
#define ISODD(x) ( ((x) % 2) != 0)

// Min and Max macros
#define MAX(x,y) ( (x) > (y) ? (x) : (y) )
#define MIN(x,y) ( (x) < (y) ? (x) : (y) )

#define ROUND(x) ( (int)( floor((x)+0.5) ) )

#define GZ_DELETE(p){ if(p) { delete (p);  (p)=NULL; } }

namespace gazebo
{

  /// \addtogroup gazebo_server
  /// \brief Global attributes.
  /// \{

  /// \brief Class to hold global attributes
  /**
  The \c param:Global tag is used to specify certain global parameters
  for the server. Set these parameters as needed in the world file E.g.:

  \verbatim
  <param:Global>
    <pause>true</pause>
  </param:Global>
  \endverbatim
  
  The parameters are as follows:
  - pause (bool)
    - Set to true to start the simulator in paused mode.
    - Default: false
  */  
  class Global
  {

    /// Paths gazebo install
    public: static std::list<std::string> gazeboPaths;

    /// Paths to the ogre install
    public: static std::list<std::string> ogrePaths;

    /// Global pose offset
    public: static Pose3d poseOffset;

    private: Global();
    private: ~Global();
  };

  /// \}
}

#endif
