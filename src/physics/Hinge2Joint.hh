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

#include "math/Angle.hh"
#include "math/Vector3.hh"
#include "physics/Joint.hh"

namespace gazebo
{
	namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{


    /// \brief A two axis hinge joint
    template< class T>
    class Hinge2Joint : public T
    {
      /// \brief Constructor
      public: Hinge2Joint() : T()
              { this->AddType(Base::HINGE2_JOINT); }
    
      /// \brief Destructor
      public: virtual ~Hinge2Joint()
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
                     // Perform this three step ordering to ensure the parameters 
                     // are set properly. This is taken from the ODE wiki.
                     this->SetHighStop(0,limitElem->GetValueDouble("upper"));
                     this->SetLowStop( 0,limitElem->GetValueDouble("lower"));
                     this->SetHighStop(0,limitElem->GetValueDouble("upper"));
                   }
 
                   if (_sdf->GetElement("axis2")->HasElement("limit"))
                   {
                     sdf::ElementPtr limitElem = _sdf->GetElement("axis2")->GetElement("limit");
                     // Perform this three step ordering to ensure the parameters 
                     // are set properly. This is taken from the ODE wiki.
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

