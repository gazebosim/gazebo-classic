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
/* Desc: A universal joint
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 */

#ifndef UNIVERSALJOINT_HH
#define UNIVERSALJOINT_HH

#include "physics/Joint.hh"

namespace gazebo
{
	namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief A universal joint
    template<class T>
    class UniversalJoint : public T
    {
      /// \brief Constructor
      public: UniversalJoint() : T()
              { this->AddType(Base::UNIVERSAL_JOINT); }
    
      /// \brief Destuctor
      public: virtual ~UniversalJoint()
              { }
    
      /// \brief Load the joint
      protected: virtual void Load( sdf::ElementPtr &_sdf )
                 {
                   T::Load(_sdf);
    
                   this->SetAxis(0,
                       _sdf->GetElement("axis")->GetValueVector3("xyz"));
                   this->SetAxis(1,
                       _sdf->GetElement("axis2")->GetValueVector3("xyz"));
    
                   if (_sdf->GetElement("axis")->HasElement("limit"))
                   {
                     sdf::ElementPtr limitElem = _sdf->GetElement("axis")->GetElement("limit");

                     // Perform this three step ordering to ensure the 
                     // parameters are set properly. 
                     // This is taken from the ODE wiki.
                     this->SetHighStop(0,limitElem->GetValueDouble("upper"));
                     this->SetLowStop( 0,limitElem->GetValueDouble("lower"));
                     this->SetHighStop(0,limitElem->GetValueDouble("upper"));
                   }

                   if (_sdf->GetElement("axis2")->HasElement("limit"))
                   {
                     sdf::ElementPtr limitElem = _sdf->GetElement("axis2")->GetElement("limit");

                     // Perform this three step ordering to ensure the 
                     // parameters  are set properly. 
                     // This is taken from the ODE wiki.
                     limitElem = _sdf->GetElement("axis2")->GetElement("limit");
                     this->SetHighStop(1,limitElem->GetValueDouble("upper"));
                     this->SetLowStop( 1,limitElem->GetValueDouble("lower"));
                     this->SetHighStop(1,limitElem->GetValueDouble("upper"));
                   }
                 }
    };
    /// \}
  }
}
#endif

