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

#ifndef _SCREWJOINT_HH_
#define _SCREWJOINT_HH_

#include "gazebo/physics/Joint.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class ScrewJoint ScrewJoint.hh physics/physics.hh
    /// \brief A screw joint, which has both  prismatic and rotational DOFs
    template<class T>
    class ScrewJoint : public T
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent of the joint.
      public: explicit ScrewJoint(BasePtr _parent) : T(_parent)
              {this->AddType(Base::SCREW_JOINT);}

      /// \brief Destructor.
      public: virtual ~ScrewJoint()
              {}

      /// \brief Load a ScreJoint.
      /// \param[in] _sdf SDF value to load from
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

      /// \brief Set the anchor.
      /// \param[in] _index Index of the axis. Not Used.
      /// \param[in] _anchor Anchor value for the joint.
      public: virtual void SetAnchor(int _index,
                                     const math::Vector3 &_anchor);

      /// \brief Get the anchor.
      /// \param[in] _index Index of the axis. Not Used.
      /// \return Anchor for the joint.
      public: virtual math::Vector3 GetAnchor(int _index) const;

      /// \brief Set screw joint thread pitch.
      ///
      /// This must be implemented in a child class
      /// \param[in] _index Index of the axis.
      /// \param[in] _threadPitch Thread pitch value.
      public: virtual void SetThreadPitch(int _index, double _threadPitch) = 0;

      /// \brief The anchor value is not used internally.
      protected: math::Vector3 fakeAnchor;

      /// \brief Pitch of the thread.
      protected: double threadPitch;
    };
    /// \}

    template<class T>
    void ScrewJoint<T>::SetAnchor(int /*_index*/,
                                  const math::Vector3 &_anchor)
    {this->fakeAnchor = _anchor;}

    template<class T>
    math::Vector3 ScrewJoint<T>::GetAnchor(int /*_index*/) const
    {return this->fakeAnchor;}
  }
}
#endif
