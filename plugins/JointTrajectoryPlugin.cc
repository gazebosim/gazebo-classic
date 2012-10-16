/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
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
/*
 * Desc: 3D position interface for ground truth.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
 */

#include <plugins/JointTrajectoryPlugin.hh>

namespace gazebo
{
/////////////////////////////////////////////////
JointTrajectoryPlugin::JointTrajectoryPlugin()
{
}

/////////////////////////////////////////////////
JointTrajectoryPlugin::~JointTrajectoryPlugin()
{
  event::Events::DisconnectWorldUpdateStart(this->updateConnection);
}

/////////////////////////////////////////////////
void JointTrajectoryPlugin::Load(physics::ModelPtr _parent,
                                 sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->world = _parent->GetWorld();
  this->model = _parent;

  // this->world->GetPhysicsEngine()->SetGravity(math::Vector3(0,0,0));
  // Get Joints
  sdf::ElementPtr jointElem = _sdf->GetElement("joint");
  while (jointElem)
  {
    // FIXME: below segfaults on second <joint></joint> entry, SDF problem?
    gzerr << jointElem->GetValueString() << "\n";

    physics::JointPtr j = this->model->GetJoint(jointElem->GetValueString());
    if (j)
    {
      this->joints.push_back(j);
    }
    else
    {
      j.reset();
    }
    jointElem = _sdf->GetNextElement("joint");
  }

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateStart(
      boost::bind(&JointTrajectoryPlugin::UpdateStates, this));
}

/////////////////////////////////////////////////
void JointTrajectoryPlugin::UpdateStates()
{
  common::Time cur_time = this->world->GetSimTime();

  for (unsigned int i = 0; i < this->joints.size(); ++i)
    this->joints[i]->SetAngle(0, cos(cur_time.Double()));
}

GZ_REGISTER_MODEL_PLUGIN(JointTrajectoryPlugin)
}
