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
/* Desc: A simbody screw or primastic joint
 * Author: Nate Koenig
 * Date: 13 Oct 2009
 */

#include "common/Console.hh"
#include "common/Exception.hh"

#include "physics/simbody/SimbodyLink.hh"
#include "physics/simbody/SimbodyPhysics.hh"
#include "physics/simbody/SimbodyTypes.hh"
#include "physics/simbody/SimbodyScrewJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyScrewJoint::SimbodyScrewJoint(MultibodySystem *_world, BasePtr _parent)
    : ScrewJoint<SimbodyJoint>(_parent)
{
  this->world = _world;
}

//////////////////////////////////////////////////
SimbodyScrewJoint::~SimbodyScrewJoint()
{
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::Load(sdf::ElementPtr _sdf)
{
  ScrewJoint<SimbodyJoint>::Load(_sdf);
  this->SetThreadPitch(0, this->threadPitch);
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::Attach(LinkPtr _one, LinkPtr _two)
{
  ScrewJoint<SimbodyJoint>::Attach(_one, _two);

  SimbodyLinkPtr simbodyChildLink =
    boost::shared_static_cast<SimbodyLink>(this->childLink);
  SimbodyLinkPtr simbodyParentLink =
    boost::shared_static_cast<SimbodyLink>(this->parentLink);

  if (!simbodyChildLink || !simbodyParentLink)
    gzthrow("Requires simbody bodies");

  // Add the joint to the world

  // Allows access to impulse
}

//////////////////////////////////////////////////
math::Angle SimbodyScrewJoint::GetAngle(int /*_index*/) const
{
  return math::Angle();
}

//////////////////////////////////////////////////
double SimbodyScrewJoint::GetVelocity(int /*_index*/) const
{
  gzerr << "Not implemented in simbody\n";
  return 0;
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetVelocity(int /*_index*/, double /*_angle*/)
{
  gzerr << "Not implemented in simbody\n";
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetAxis(int /*_index*/, const math::Vector3 &/*_axis*/)
{
  gzerr << "Not implemented in simbody\n";
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetDamping(int /*index*/, double /*_damping*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetThreadPitch(int /*_index*/, double /*_threadPitch*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetForce(int /*_index*/, double /*_force*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetHighStop(int /*_index*/, const math::Angle &_angle)
{
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetLowStop(int /*_index*/, const math::Angle &_angle)
{
}

//////////////////////////////////////////////////
math::Angle SimbodyScrewJoint::GetHighStop(int /*_index*/)
{
  return math::Angle();
}

//////////////////////////////////////////////////
math::Angle SimbodyScrewJoint::GetLowStop(int /*_index*/)
{
  return math::Angle();
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetMaxForce(int /*_index*/, double /*_force*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
double SimbodyScrewJoint::GetMaxForce(int /*index*/)
{
  gzerr << "Not implemented\n";
  return 0;
}

//////////////////////////////////////////////////
math::Vector3 SimbodyScrewJoint::GetGlobalAxis(int /*_index*/) const
{
  gzerr << "SimbodyScrewJoint::GetGlobalAxis not implemented\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
math::Angle SimbodyScrewJoint::GetAngleImpl(int /*_index*/) const
{
  gzerr << "SimbodyScrewJoint::GetAngleImpl not implemented\n";
  return math::Angle();
}
