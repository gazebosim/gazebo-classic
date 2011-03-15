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
/* Desc: A slider or primastic joint
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id$
 */

#ifndef SLIDERJOINT_HH
#define SLIDERJOINT_HH

#include <float.h>
#include "common/Param.hh"
#include "Joint.hh"

namespace gazebo
{
	namespace physics
  {
  
  /// \addtogroup gazebo_physics_joints
  /// \{
  /** \defgroup gazebo_slider_joint Slider Joint
    
    \brief A slider joint
  
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
    <joint:slider name="slider_joint>
      <body1>body1_name</body1>
      <body2>body2_name</body2>
      <anchor>anchor_body</anchor>
      <axis>0 0 1</axis>
      <lowStop>0</lowStop>
      <highStop>30</highStop>
    </joint:slider>
    \endverbatim
  */
  /// \}
  
  
  /// \addtogroup gazebo_slider_joint Slider Joint
  /// \{
  
    /// \brief A slider joint
    template<class T>
    class SliderJoint : public T
    {
      /// \brief Constructor
      public: SliderJoint( ) : T()
              {
                this->AddType(SLIDER_JOINT);
  
                common::Param::Begin(&this->parameters);
                this->axisP = new common::ParamT<common::Vector3>("axis",common::Vector3(0,0,1), 0);
                this->loStopP = new common::ParamT<double>("lowStop",-DBL_MAX,0);
                this->hiStopP = new common::ParamT<double>("highStop",DBL_MAX,0);
                this->dampingP = new common::ParamT<double>("damping",0.0, 0);
                common::Param::End();
              } 
  
      /// \brief Destructor
      public: virtual ~SliderJoint()
              {
                delete this->axisP;
                delete this->loStopP;
                delete this->hiStopP;
                delete this->dampingP;
              }
  
      /// \brief Load the joint
      protected: virtual void Load(common::XMLConfigNode *node)
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
      public: virtual void SetAnchor( int index, const common::Vector3 &anchor) {fakeAnchor = anchor;}
  
      /// \brief Get the anchor
      public: virtual common::Vector3 GetAnchor(int index) const {return fakeAnchor;}
   
      protected: common::ParamT<common::Vector3> *axisP;
      protected: common::ParamT<double> *loStopP;
      protected: common::ParamT<double> *hiStopP; 
      protected: common::ParamT<double> *dampingP; 
      protected: common::Vector3 fakeAnchor;
    };
    
  /// \}
  }
}
#endif
