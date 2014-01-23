/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
/* Desc: A screw or primastic/rotational joint
 * Author: Nate Koenig, John Hsu
 * Date: 21 May 2003
 */

#ifndef _SCREWJOINT_HH_
#define _SCREWJOINT_HH_

#include "gazebo/physics/Joint.hh"
#include "gazebo/common/Console.hh"

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
      public: explicit ScrewJoint(BasePtr _parent) : T(_parent), threadPitch(0)
              {this->AddType(Base::SCREW_JOINT);}

      /// \brief Destructor.
      public: virtual ~ScrewJoint()
              { }

      /// \interal
      public: virtual unsigned int GetAngleCount() const
              {return 2;}

      /// \brief Load a ScrewJoint.
      /// \param[in] _sdf SDF value to load from
      public: virtual void Load(sdf::ElementPtr _sdf)
                 {
                   T::Load(_sdf);

                   if (_sdf->HasElement("thread_pitch"))
                   {
                     this->threadPitch =
                       _sdf->GetElement("thread_pitch")->Get<double>();
                   }
                   else
                   {
                     this->threadPitch = 1.0;
                     gzwarn << "<thread_pitch> element not specified. "
                            << "Using default value of "
                            << this->threadPitch
                            << ". \n";
                   }

                   if (_sdf->HasElement("axis"))
                   {
                     sdf::ElementPtr axisElem = _sdf->GetElement("axis");
                     this->SetAxis(0, axisElem->Get<math::Vector3>("xyz"));
                     if (axisElem->HasElement("limit"))
                     {
                       sdf::ElementPtr limitElem =
                         _sdf->GetElement("axis")->GetElement("limit");

                       // Perform this three step ordering to ensure the
                       // parameters are set properly. This is taken from
                       // the ODE wiki.
                       this->SetHighStop(0, limitElem->Get<double>("upper"));
                       this->SetLowStop(0, limitElem->Get<double>("lower"));
                       this->SetHighStop(0, limitElem->Get<double>("upper"));
                     }
                   }
                 }

      /// \brief Set the anchor.
      /// \param[in] _index Index of the axis. Not Used.
      /// \param[in] _anchor Anchor value for the joint.
      public: virtual void SetAnchor(int _index, const math::Vector3 &_anchor);

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

      /// \brief Get screw joint thread pitch.
      ///
      /// This must be implemented in a child class
      /// \param[in] _index Index of the axis.
      /// \return _threadPitch Thread pitch value.
      public: virtual double GetThreadPitch(unsigned int _index) = 0;

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
