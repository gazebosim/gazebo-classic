/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig
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
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#ifndef SCREWJOINT_HH
#define SCREWJOINT_HH

#include <float.h>
#include "physics/Joint.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief A screw joint
    template<class T>
    class ScrewJoint : public T
    {
      /// \brief Constructor
      public: ScrewJoint(BasePtr _parent) : T(_parent)
              { this->AddType(Base::SCREW_JOINT); }
      /// \brief Destructor
      public: virtual ~ScrewJoint()
              { }
      /// \brief Load a ScreJoint
      protected: virtual void Load(sdf::ElementPtr _sdf)
                 {
                   T::Load(_sdf);

                   if (_sdf->HasElement("thread_pitch"))
                   {
                     this->threadPitch =
                       _sdf->GetElement("thread_pitch")->GetValueDouble();
                   }
                   else
                   {
                     gzerr << "should not see this\n";
                     this->threadPitch = 1.0;
                   }

                   if (_sdf->HasElement("axis"))
                   {
                     sdf::ElementPtr axisElem = _sdf->GetElement("axis");
                     this->SetAxis(0, axisElem->GetValueVector3("xyz"));
                     if (axisElem->HasElement("limit"))
                     {
                       sdf::ElementPtr limitElem =
                         _sdf->GetElement("axis")->GetElement("limit");

                       // Perform this three step ordering to ensure the
                       // parameters are set properly. This is taken from
                       // the ODE wiki.
                       this->SetHighStop(0, limitElem->GetValueDouble("upper"));
                       this->SetLowStop(0, limitElem->GetValueDouble("lower"));
                       this->SetHighStop(0, limitElem->GetValueDouble("upper"));
                     }
                   }
                 }

      /// \brief Set the anchor
      public: virtual void SetAnchor(int /*_index */,
                                     const math::Vector3 &anchor)
              {fakeAnchor = anchor;}

      /// \brief Get the anchor
      public: virtual math::Vector3 GetAnchor(int /*_index*/) const
               {return fakeAnchor;}

      protected: math::Vector3 fakeAnchor;
      protected: double threadPitch;
    };
    /// \}
  }
}
#endif


