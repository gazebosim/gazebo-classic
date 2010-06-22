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
 * Desc: Bumper Controller
 * Author: Nate Koenig
 * Date: 09 Sept 2008
 */
#ifndef GENERICBUMPER_CONTROLLER_HH
#define GENERICBUMPER_CONTROLLER_HH

#include <sys/time.h>

#include "Controller.hh"
#include "Entity.hh"

namespace gazebo
{
  class ContactSensor;

  /// \addtogroup gazebo_controller
  /// \{
  /** \defgroup bumper bumper
  
    \brief A controller that returns bump contacts
  
    \{
  */
  
  /// \brief A Bumper controller
  class Generic_Bumper : public Controller
  {
    /// Constructor
      public: Generic_Bumper(Entity *parent );
  
    /// Destructor
      public: virtual ~Generic_Bumper();
  
    /// Load the controller
    /// \param node XML config node
    /// \return 0 on success
    protected: virtual void LoadChild(XMLConfigNode *node);
  
    /// Init the controller
    /// \return 0 on success
    protected: virtual void InitChild();
  
    /// Update the controller
    /// \return 0 on success
    protected: virtual void UpdateChild();
  
    /// Finalize the controller
    /// \return 0 on success
    protected: virtual void FiniChild();
  
    /// The parent Model
    private: ContactSensor *myParent;
  
    /// The Iface. 
    private: libgazebo::BumperIface *myIface;
  };
  
  /** \} */
  /// \}

}

#endif

