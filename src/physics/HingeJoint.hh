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

#include "math/Angle.hh"
#include "math/Vector3.hh"
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
              { this->AddType(Base::HINGE_JOINT); }
   
      ///  \brief Destructor
      public: virtual ~HingeJoint()
              { }
  
      /// \brief Load joint
      protected: virtual void Load( sdf::ElementPtr &_sdf)
                 {
                   T::Load(_sdf);

                   if (_sdf->HasElement("axis"))
                   {
                     sdf::ElementPtr axisElem = _sdf->GetElement("axis");
                     this->SetAxis(0, axisElem->GetValueVector3("xyz"));
                     if (axisElem->HasElement("limit"))
                     {
                       sdf::ElementPtr limitElem = _sdf->GetElement("axis")->GetElement("limit");

                       // Perform this three step ordering to ensure the 
                       // parameters are set properly. 
                       // This is taken from the ODE wiki.
                       this->SetHighStop(0,limitElem->GetValueDouble("upper"));
                       this->SetLowStop( 0,limitElem->GetValueDouble("lower"));
                       this->SetHighStop(0,limitElem->GetValueDouble("upper"));
                     }
                   }

                 }
      };
  }
}
#endif

