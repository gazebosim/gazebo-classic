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
/* Desc: A body that has a box shape
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 */

#ifndef HINGEJOINT_HH
#define HINGEJOINT_HH

#include "common/Angle.hh"
#include "common/Vector3.hh"
#include "common/Param.hh"
#include "common/XMLConfig.hh"
#include "common/Global.hh"

namespace gazebo
{
	namespace physics
  {
    ///\brief A single axis hinge joint
    template<class T>
    class HingeJoint : public T
    {
      /// \brief Constructor
      public: HingeJoint() : T()
              {
                this->AddType(Base::HINGE_JOINT);
  
                common::Param::Begin(&this->parameters);
                this->axisP = new common::ParamT<common::Vector3>("axis",common::Vector3(0,1,0), 1);
                this->loStopP = new common::ParamT<common::Angle>("lowStop",-std::numeric_limits<float>::max(),0);
                this->hiStopP = new common::ParamT<common::Angle>("highStop",std::numeric_limits<float>::max(),0);
                this->dampingP = new common::ParamT<double>("damping",0.0, 0);
                common::Param::End();
              }
   
      ///  \brief Destructor
      public: virtual ~HingeJoint()
              {
                delete this->axisP;
                delete this->loStopP;
                delete this->hiStopP;
                delete this->dampingP;
              }
  
      /// \brief Load joint
      protected: virtual void Load(common::XMLConfigNode *node)
                 {
                   this->axisP->Load(node);
                   this->loStopP->Load(node);
                   this->hiStopP->Load(node);
                   this->dampingP->Load(node);
  
                   T::Load(node);
  
                   // Perform this three step ordering to ensure the parameters 
                   // are set properly. This is taken from the ODE wiki.
                   this->SetHighStop(0, this->hiStopP->GetValue());
                   this->SetLowStop(0,this->loStopP->GetValue());
                   this->SetHighStop(0, this->hiStopP->GetValue());
                   //this->SetDamping(0, this->dampingP->GetValue()); // uncomment when opende damping is tested and ready
  
                   common::Vector3 a = **this->axisP;
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
  
      protected: common::ParamT<common::Vector3> *axisP;
      protected: common::ParamT<common::Angle> *loStopP;
      protected: common::ParamT<common::Angle> *hiStopP; 
      protected: common::ParamT<double> *dampingP; 
    };
  }
}
#endif

