/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include "physics/physics.hh"
#include "transport/transport.hh"
#include "plugins/TestPlugin.hh"

using namespace gazebo;
GZ_REGISTER_WORLD_PLUGIN(TestPlugin)

/////////////////////////////////////////////////
TestPlugin::TestPlugin()
{
	this->joints_created = false;
}

/////////////////////////////////////////////////
void TestPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/)
{
	std::cout << "loading dynamic joint plugin" << std::endl;

    // Store the pointer to the world
	  this->world = _world;

	  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			  boost::bind(&TestPlugin::OnUpdate, this));

}

void TestPlugin::OnUpdate()
{
	// get models / links after a few seconds, in order to avoid:
	// 'Error [ConnectionManager.cc:123] Connection Manager is not running'
	// when loading the plugin

	if (this->world->GetSimTime().sec > 2 && !this->joints_created)
	{
		this->liquid_model = this->world->GetModel("liquid_spheres");

		std::cout <<  this->liquid_model->GetName().c_str() << std::endl;

		this->center_link = this->liquid_model->GetLink("sphere_link_0");

		// loading the links of the model
		this->myLinks = this->liquid_model->GetLinks();

		std::cout << this->myLinks.size() << std::endl;

		//start from link 1, because 0 is the center
		for (unsigned int i = 1; i < this->myLinks.size(); i++)
		{
			std::cout << this->myLinks[i]->GetName().c_str() << std::endl;

			this->CreateJoint(this->center_link,
					this->myLinks[i]);
		}

		this->joints_created = true;
	}
}

void TestPlugin::CreateJoint(physics::LinkPtr _centerLink,
    physics::LinkPtr _extLink)
{
  math::Vector3 axis,direction;

  // compute the axis
  direction = _extLink->GetWorldPose().pos - _centerLink->GetWorldPose().pos;
  direction.Normalize();
  axis = direction.Cross(math::Vector3(0.0, 0.0, 1.0));

  this->myJoints.push_back(
		  this->world->GetPhysicsEngine()->CreateJoint(
        "revolute", this->liquid_model));

  this->myJoints.back()->Attach(_extLink, _centerLink );

  this->myJoints.back()->Load(_extLink, _centerLink,
      math::Pose(_centerLink->GetWorldPose().pos, math::Quaternion()));

  this->myJoints.back()->SetAxis(0, axis);

  /*this->myJoints.back()->SetAttribute("stop_cfm",0, this->stop_cfm);
  this->myJoints.back()->SetAttribute("stop_erp",0, this->stop_erp);
  this->myJoints.back()->SetAttribute("hi_stop",0, this->limit_stop);
  this->myJoints.back()->SetAttribute("lo_stop",0, - this->limit_stop);
  */
}
