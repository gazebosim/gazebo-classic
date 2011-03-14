/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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

#include "Param.hh"
#include "Controller.hh"

namespace gazebo
{
  class Joint;

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
    protected: virtual void LoadChild(XMLConfigNode *node);

    /// \brief Save the controller.
    /// \stream Output stream
    protected: void SaveChild(std::string &prefix, std::ostream &stream);

    /// \brief Init the controller
    protected: virtual void InitChild();
  
    /// \brief Update the controller
    protected: virtual void UpdateChild();
  
    /// \brief Finalize the controller
    protected: virtual void FiniChild();
              
    /// \brief Reset the controller
    protected: virtual void ResetChild();
  
    /// \brief Put camera data to the iface
    private: void PutPTZData();
  
    /// The camera interface
    private: libgazebo::PTZIface *ptzIface;
  
    /// The parent sensor
    private: Model *myParent;

    /// Pan joint
    private: Joint *panJoint;

    /// Tilt joint
    private: Joint *tiltJoint;

    private: float cmdTilt;
    private: float cmdPan;

    private: ParamT<double> *motionGainP;
    private: ParamT<double> *forceP;

    private: ParamT<std::string> *panJointNameP;
    private: ParamT<std::string> *tiltJointNameP;
  };
  
  /** /} */
  /// @}

}

#endif

