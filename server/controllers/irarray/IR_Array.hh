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
 * Desc: IR array controller.
 * Author: Wenguo Liu
 * Date: 24 Apr 2008
 */

#ifndef IR_ARRAY_HH
#define IR_ARRAY_HH

#include "Controller.hh"

namespace gazebo
{
  class IRIface;
  class IRSensor;


  /// \brief Sick LMS 200 laser controller.
  /// 
  /// This is a controller that simulates a ir array
  class IR_Array : public Controller
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public: IR_Array(Entity *parent);
  
    /// \brief Destructor
    public: virtual ~IR_Array();
  
    /// \brief Load the controller
    /// \param node XML config node
    /// \return 0 on success
    protected: virtual void LoadChild(XMLConfigNode *node);
  
    /// \brief Init the controller
    /// \return 0 on success
    protected: virtual void InitChild();
  
    /// \brief Update the controller
    /// \return 0 on success
    protected: virtual void UpdateChild();
  
    /// \brief Finalize the controller
    /// \return 0 on success
    protected: virtual void FiniChild();
  
    /// \brief Put laser data to the iface
    private: void PutIRData();
  
    /// The ir interface
    private: IRIface *irIface;
  
    /// The parent sensor
    private: IRSensor *myParent;
  
  };
  
  /** /} */
  /// @}
}

#endif
