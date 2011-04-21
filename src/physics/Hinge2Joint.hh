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
/* Desc: A hinge joint with 2 degrees of freedom
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 */

#ifndef HINGE2JOINT_HH
#define HINGE2JOINT_HH

#include "common/Param.hh"
#include "common/Angle.hh"
#include "common/Vector3.hh"
#include "physics/Joint.hh"

namespace gazebo
{
	namespace physics
  {
   /// \brief A two axis hinge joint
  template< class T>
  class Hinge2Joint : public T
  {
    /// \brief Constructor
    public: Hinge2Joint() : T()
            {
              this->AddType(Base::HINGE2_JOINT);
  
              common::Param::Begin(&this->parameters);
              this->axis1P = new common::ParamT<common::Vector3>("axis1",common::Vector3(0,0,1), 0);
              this->axis2P = new common::ParamT<common::Vector3>("axis2",common::Vector3(0,0,1), 0);
              this->loStop1P = new common::ParamT< common::Angle>("lowStop1",-M_PI,0);
              this->hiStop1P = new common::ParamT< common::Angle>("highStop1",M_PI,0);
              this->loStop2P = new common::ParamT< common::Angle>("lowStop2",-M_PI,0);
              this->hiStop2P = new common::ParamT< common::Angle>("highStop2",M_PI,0);
              common::Param::End();
            }
  
    /// \brief Destructor
    public: virtual ~Hinge2Joint()
            {
              delete this->axis1P;
              delete this->axis2P;
              delete this->loStop1P;
              delete this->hiStop1P;
              delete this->loStop2P;
              delete this->hiStop2P;
            }
  
    /// \brief Load the joint
    protected: virtual void Load(common::XMLConfigNode *node)
               {
                 this->axis1P->Load(node);
                 this->axis2P->Load(node);
                 this->loStop1P->Load(node);
                 this->hiStop1P->Load(node);
                 this->loStop2P->Load(node);
                 this->hiStop2P->Load(node);
  
                 T::Load(node);
  
                 this->SetAxis(0,**(this->axis1P));
                 this->SetAxis(1,**(this->axis2P));
  
                 // Perform this three step ordering to ensure the parameters 
                 // are set properly. This is taken from the ODE wiki.
                 this->SetHighStop(0,**this->hiStop1P);
                 this->SetLowStop(0,**this->loStop1P);
                 this->SetHighStop(0,**this->hiStop1P);
  
                 // Perform this three step ordering to ensure the parameters 
                 // are set properly. This is taken from the ODE wiki.
                 this->SetHighStop(1,**this->hiStop2P);
                 this->SetLowStop(1,**this->loStop2P);
                 this->SetHighStop(1,**this->hiStop2P);
               }
  
    /// \brief Save a joint to a stream in XML format
    protected: virtual void SaveJoint(std::string &prefix, std::ostream &stream)
               {
                 T::SaveJoint(prefix, stream);
                 stream << prefix << *(this->axis1P) << "\n";
                 stream << prefix << *(this->loStop1P) << "\n";
                 stream << prefix << *(this->hiStop1P) << "\n";
  
                 stream << prefix << *(this->axis2P) << "\n";
                 stream << prefix << *(this->loStop2P) << "\n";
                 stream << prefix << *(this->hiStop2P) << "\n";
  
               }
  
    protected: common::ParamT<common::Vector3> *axis1P;
    protected: common::ParamT<common::Angle> *loStop1P;
    protected: common::ParamT<common::Angle> *hiStop1P; 
  
    protected: common::ParamT<common::Vector3> *axis2P;
    protected: common::ParamT<common::Angle> *loStop2P;
    protected: common::ParamT<common::Angle> *hiStop2P; 
  };
  }
}
#endif

