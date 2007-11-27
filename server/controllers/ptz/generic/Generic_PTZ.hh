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
 * Desc: A generic ptz controller
 * Author: Nathan Koenig
 * Date: 26 Nov 2007
 * SVN: $Id$
 */

#ifndef GENERIC_PTZ_HH
#define GENERIC_PTZ_HH

#include "Controller.hh"

namespace gazebo
{
  class HingeJoint;
  class PTZIface;

/// @addtogroup gazebo_controller
/// @{
/** \defgroup genericptz generic ptz

  \brief Generic pan-tilt-zoom controller.
  
  This is a controller that controls a pan, tilt, zoom unit 

  \verbatim
  <model:physical name="ptz_model">
    <body:empty name="ptz_body">
      <controller:generic_ptz name="controller-name">
        <interface:ptz name="iface-name"/>
      </controller:generic_ptz>
    </body:empty>
  </model:phyiscal>
  \endverbatim
 
\{
*/

  /// \brief Generic ptz controller.
  /// 
  /// This is a controller for a generic PTZ
  class Generic_PTZ : public Controller
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public: Generic_PTZ(Entity *parent);
  
    /// \brief Destructor
    public: virtual ~Generic_PTZ();
  
    /// \brief Load the controller
    /// \param node XML config node
    /// \return 0 on success
    protected: virtual void LoadChild(XMLConfigNode *node);
  
    /// \brief Init the controller
    /// \return 0 on success
    protected: virtual void InitChild();
  
    /// \brief Update the controller
    /// \return 0 on success
    protected: virtual void UpdateChild(UpdateParams &params);
  
    /// \brief Finalize the controller
    /// \return 0 on success
    protected: virtual void FiniChild();
  
    /// \brief Put camera data to the iface
    private: void PutPTZData();
  
    /// The camera interface
    private: PTZIface *ptzIface;
  
    /// The parent sensor
    private: Model *myParent;

    /// Pan joint
    private: HingeJoint *panJoint;

    /// Tilt joint
    private: HingeJoint *tiltJoint;

    private: float motionGain;
    private: float cmdTilt;
    private: float cmdPan;
  };
  
  /** /} */
  /// @}

}

#endif

