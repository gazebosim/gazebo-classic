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
/* Desc: A screw or primastic/rotational joint
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id$
 */

#ifndef SCREWJOINT_HH
#define SCREWJOINT_HH

#include <float.h>
#include "Param.hh"
#include "Joint.hh"

namespace gazebo
{

/// \addtogroup gazebo_physics_joints
/// \{
/** \defgroup gazebo_screw_joint Screw Joint
  
  \brief A screw joint

  \par Attributes
  - body1 (string)
    - Name of the first body to attach to the joint
  - body2 (string)
    - Name of the second body to attach to the joint
  - anchor (string)
    - Name of the body which will act as the anchor to the joint
  - axis (float, tuple)
    - Defines the axis of movement
    - Default: 0 0 1
  - lowStop (float, meters)
    - The low stop position
    - Default: infinity
  - highStop (float, meters)
    - The high stop position
    - Default: infinity
  - erp (double)
    - Error reduction parameter. 
    - Default = 0.4
  - cfm (double)
    - Constraint force mixing. 
    - Default = 0.8


  \par Example
  \verbatim
  <joint:screw name="screw_joint>
    <body1>body1_name</body1>
    <body2>body2_name</body2>
    <anchor>anchor_body</anchor>
    <axis>0 0 1</axis>
    <lowStop>0</lowStop>
    <highStop>30</highStop>
  </joint:screw>
  \endverbatim
*/
/// \}


/// \addtogroup gazebo_screw_joint Screw Joint
/// \{

  /// \brief A screw joint
  template<class T>
  class ScrewJoint : public T
  {
    /// \brief Constructor
    public: ScrewJoint( ) : T()
            {
              this->type = Joint::SCREW;

              Param::Begin(&this->parameters);
              this->axisP = new ParamT<Vector3>("axis",Vector3(0,0,1), 0);
              this->loStopP = new ParamT<double>("lowStop",-DBL_MAX,0);
              this->hiStopP = new ParamT<double>("highStop",DBL_MAX,0);
              this->dampingP = new ParamT<double>("damping",0.0, 0);
              Param::End();
            } 

    /// \brief Destructor
    public: virtual ~ScrewJoint()
            {
              delete this->axisP;
              delete this->loStopP;
              delete this->hiStopP;
              delete this->dampingP;
            }

    /// \brief Load the joint
    protected: virtual void Load(XMLConfigNode *node)
               {
                 this->axisP->Load(node);
                 this->loStopP->Load(node);
                 this->hiStopP->Load(node);
                 this->dampingP->Load(node);

                 T::Load(node);

                 this->SetAxis(0, **(this->axisP));

                 // Perform this three step ordering to ensure the parameters 
                 // are set properly. This is taken from the ODE wiki.
                 this->SetHighStop(0,**(this->hiStopP));
                 this->SetLowStop(0,**(this->loStopP));
                 this->SetHighStop(0,**(this->hiStopP));
                 //this->SetDamping(0, this->dampingP->GetValue()); // uncomment when opende damping is tested and ready
               }
  
    /// \brief Save a joint to a stream in XML format
    protected: virtual void SaveJoint(std::string &prefix, std::ostream &stream)
               {
                 T::SaveJoint(prefix, stream);
                 stream << prefix << *(this->axisP) << "\n";
                 stream << prefix << *(this->loStopP) << "\n";
                 stream << prefix << *(this->hiStopP) << "\n";
               }
    /// \brief Set the anchor
    public: virtual void SetAnchor( int index, const Vector3 &anchor) {fakeAnchor = anchor;}

    /// \brief Get the anchor
    public: virtual Vector3 GetAnchor(int index) const {return fakeAnchor;}
 
    protected: ParamT<Vector3> *axisP;
    protected: ParamT<double> *loStopP;
    protected: ParamT<double> *hiStopP; 
    protected: ParamT<double> *dampingP; 
    protected: Vector3 fakeAnchor;
  };
  
/// \}
}
#endif
