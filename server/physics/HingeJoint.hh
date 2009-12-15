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
/* Desc: A body that has a box shape
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id$
 */

#ifndef HINGEJOINT_HH
#define HINGEJOINT_HH

#include "Angle.hh"
#include "Vector3.hh"
#include "Param.hh"
#include "Joint.hh"
#include "XMLConfig.hh"
#include "World.hh"
#include "Global.hh"

namespace gazebo
{

  /// \addtogroup gazebo_physics_joints
  /// \{
  /** \defgroup gazebo_hinge_joint Hinge Joint
    
    \brief A two-axis hinge joint.
  
    \par Attributes
    - body1 (string)
      - Name of the first body to attach to the joint
    - body2 (string)
      - Name of the second body to attach to the joint
    - anchor (string)
      - Name of the body which will act as the anchor to the joint
    - axis (float, tuple)
      - Defines the axis of rotation for the first degree of freedom
      - Default: 0 0 1
    - lowStop (float, degrees)
      - The low stop angle for the first degree of freedom
      - Default: infinity
    - highStop (float, degrees)
      - The high stop angle for the first degree of freedom
      - Default: infinity
    - erp (double)
      - Error reduction parameter. 
      - Default = 0.4
    - cfm (double)
      - Constraint force mixing. 
      - Default = 0.8
  
    \par Example
    \verbatim
    <joint:hinge name="hinge_joint>
      <body1>body1_name</body1>
      <body2>body2_name</body2>
      <anchor>anchor_body</anchor>
      <axis>0 0 1</axis>
      <lowStop>0</lowStop>
      <highStop>30</highStop>
    </joint:hinge>
    \endverbatim
  */
  /// \}
  
  /// \addtogroup gazebo_hinge_joint
  /// \{
  
  ///\brief A single axis hinge joint
  template<class T>
  class HingeJoint : public T
  {
    /// \brief Constructor
    public: HingeJoint() : T()
            {
              this->type = Joint::HINGE;

              Param::Begin(&this->parameters);
              this->axisP = new ParamT<Vector3>("axis",Vector3(0,1,0), 1);
              this->loStopP = new ParamT<Angle>("lowStop",-M_PI,0);
              this->hiStopP = new ParamT<Angle>("highStop",M_PI,0);
              Param::End();
            }
 
    ///  \brief Destructor
    public: virtual ~HingeJoint()
            {
              delete this->axisP;
              delete this->loStopP;
              delete this->hiStopP;
            }

    /// \brief Load joint
    protected: virtual void Load(XMLConfigNode *node)
               {
                 this->axisP->Load(node);
                 this->loStopP->Load(node);
                 this->hiStopP->Load(node);

                 T::Load(node);

                 // Perform this three step ordering to ensure the parameters 
                 // are set properly. This is taken from the ODE wiki.
                 this->SetHighStop(0, this->hiStopP->GetValue());
                 this->SetLowStop(0,this->loStopP->GetValue());
                 this->SetHighStop(0, this->hiStopP->GetValue());

                 Vector3 a = **this->axisP;
                 this->SetAxis(0, a);
               }
 
    /// \brief Save a joint to a stream in XML format
    protected: virtual void SaveJoint(std::string &prefix, std::ostream &stream)
               {
                 T::SaveJoint(prefix, stream);
                 stream << prefix << *(this->axisP) << "\n";
                 stream << prefix << *(this->loStopP) << "\n";
                 stream << prefix << *(this->hiStopP) << "\n";
               }

    protected: ParamT<Vector3> *axisP;
    protected: ParamT<Angle> *loStopP;
    protected: ParamT<Angle> *hiStopP; 
  };
  /// \}
}
#endif

