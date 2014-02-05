/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include <gazebo/math/gzmath.hh>
#include <gazebo/physics/physics.hh>
#include "RobocupRulesPlugin.hh"

using namespace gazebo;

#define FIELD_WIDTH 20.0
#define FIELD_HEIGHT 30.0
#define TEAM_LEFT 0u
#define TEAM_RIGHT 1u
#define FREE_KICK_MOVE_DIST 15.15
#define FREE_KICK_DIST 9.15

const math::Box FieldLeft(
    math::Vector3(-FIELD_HEIGHT*0.5, -FIELD_WIDTH*0.5, 0),
    math::Vector3(0, FIELD_WIDTH*0.5, 0));

const math::Box FieldRight(
    math::Vector3(0, -FIELD_WIDTH*0.5, 0),
    math::Vector3(FIELD_HEIGHT*0.5, FIELD_WIDTH*0.5, 0));

GZ_REGISTER_WORLD_PLUGIN(RobocupRulesPlugin)

/////////////////////////////////////////////////
RobocupRulesPlugin::RobocupRulesPlugin()
{
}

/////////////////////////////////////////////////
RobocupRulesPlugin::~RobocupRulesPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
}

/////////////////////////////////////////////////
void RobocupRulesPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  sdf::ElementPtr elem;

  this->world = _world;

  // Get a pointer to the soccer ball
  this->ball = this->world->GetModel(_sdf->Get<std::string>("ball"));

  if (!this->ball)
  {
    gzerr << "Unable to find the soccer ball with name[" << 
      _sdf->Get<std::string>("ball") << "]\n";
    return;
  }

  // Make sure the ball is at the center of the field
  this->ball->SetWorldPose(math::Pose(0, 0, 0, 0, 0, 0));

  // Load all the teams
  sdf::ElementPtr teamElem = _sdf->GetElement("team");
  while (teamElem)
  {
    // Create a new team
    Team *team = new Team;
    this->teams.push_back(team);

    // Set the team name
    team->name = teamElem->Get<std::string>("name");

    // Get all the team members.
    sdf::ElementPtr memberElem = teamElem->GetElement("member");
    while (memberElem)
    {
      // Get the team member name
      std::string memberName = memberElem->Get<std::string>(); 
      physics::ModelPtr member = this->world->GetModel(memberName);
      if (member)
        team->members.push_back(member);
      else
        gzerr << "Unable to get team member with name[" << memberName << "]\n";
      memberElem = memberElem->GetNextElement("member");
    }

    teamElem = teamElem->GetNextElement("team");
  }

  // Make sure that we have two teams.
  if (this->teams.size() != 2)
  {
    gzerr << "Invalid number of teams[" << this->teams.size() 
      << "] for soccer.\n";
    return;
  }

  // Connect to the update event.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&RobocupRulesPlugin::UpdateStates, this, _1));
}

/////////////////////////////////////////////////
void RobocupRulesPlugin::Init()
{
}

/////////////////////////////////////////////////
void RobocupRulesPlugin::UpdateStates(const common::UpdateInfo & /*_info*/)
{
  this->teams[0]->members[0]->SetLinearVel(math::Vector3(1, 0, 0));
}

/////////////////////////////////////////////////
void RobocupRulesPlugin::ClearPlayers(const math::Box &_box, double _minDist, 
    unsigned int _teamIndex)
{
  if (_teamIndex >= this->teams.size())
  {
    gzerr << "Invalid team index[" << _teamIndex << "]. "
      << "Max value is[" << this->teams.size() - 1 << "]\n";
    return;
  }

  for (std::vector<physics::ModelPtr>::iterator iter =
      this->teams[_teamIndex]->members.begin();
      iter != this->teams[_teamIndex]->members.end(); ++iter)
  {
    if ((*iter)->GetBoundingBox().Intersects(_box))
    {
      // Get the current pose of the member
      math::Pose pose = (*iter)->GetWorldPose();

      // If the member is on the LEFT team, move the member's X position to
      // the LEFT
      if (_teamIndex == TEAM_LEFT)
      {
        pose.pos.x = _box.min.x -
          math::Rand::GetDblUniform(_minDist, _minDist * 2.0);
      }
      else
      {
        pose.pos.x = _box.max.x +
          math::Rand::GetDblUniform(_minDist, _minDist * 2.0);
      }

      (*iter)->SetWorldPose(pose);
    }
  }
}

/////////////////////////////////////////////////
void KickoffState::Init()
{
  // this->rules->SetPaused(true);
  // this->rules->ClearPlayers(FieldRight, FREE_KICK_MOVE_DIST, TEAM_LEFT);
  // this->rules->ClearPlayers(FieldLeft, FREE_KICK_MOVE_DIST, TEAM_RIGHT);
}
