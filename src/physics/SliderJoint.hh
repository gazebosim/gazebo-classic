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
 */

#ifndef SLIDERJOINT_HH
#define SLIDERJOINT_HH

#include <float.h>
#include "common/Param.hh"
#include "common/XMLConfig.hh"
#include "physics/Joint.hh"

namespace gazebo
{
	namespace physics
  {
 
    /// \brief A slider joint
    template<class T>
    class SliderJoint : public T
    {
      /// \brief Constructor
      public: SliderJoint( ) : T()
              {
                this->AddType(Base::SLIDER_JOINT);
  
                common::Param::Begin(&this->parameters);
                this->axisP = new common::ParamT<common::Vector3>("xyz",common::Vector3(0,0,1), 0);
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
                   this->axisP->Load(node->GetChild("axis"));
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
    
  }
}
#endif
