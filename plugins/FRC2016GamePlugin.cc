/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Subscriber.hh"
#include "gazebo/common/Events.hh"
#include "plugins/FRC2016GamePlugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(FRC2016GamePlugin)

/////////////////////////////////////////////////
FRC2016GamePlugin::FRC2016GamePlugin()
{
}

/////////////////////////////////////////////////
void FRC2016GamePlugin::Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/)
{
  GZ_ASSERT(_world, "FRC2016GamePlugin world pointer is NULL");
  this->world = _world;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->gates.push_back(
      Gate(ignition::math::Box(1.31, 2.29, 0, 2.31, 3.29, 1.0), 0));

  this->gamePieces = {"ground_plane", "field", "ball1", "ball2", "ball3",
    "ball4", "ball5", "ball6", "ball7", "ball8", "ball9", "ball10", "ball11",
    "ball12", "red_lowbar", "red_chevaldefrise", "red_moat",
    "red_drawbridge", "red_rockwall", "blue_rough", "blue_portcullis",
    "blue_ramparts", "blue_sallyport", "blue_lowbar"};

  this->launchPoses[RED] = {-10, 0, 2, 0, 0, 0};
  this->launchPoses[BLUE] = {10, 0, 2, 0, 0, 0};

  this->planes[RED].Set(ignition::math::Vector3d(0, 1, 0), -2.75);
  this->planes[BLUE].Set(ignition::math::Vector3d(0, 1, 0), 2.75);

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&FRC2016GamePlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void FRC2016GamePlugin::UpdateCastleBalls(const physics::Model_V &_models)
{
  for (auto const &model : _models)
  {
    // Skip non-balls
    if (model->GetName().find("ball") == std::string::npos)
      continue;

    if (model->GetWorldPose().Ign().Pos().Y() <= -10 &&
        (this->world->GetSimTime() - this->launchTimes[RED]) >
        common::Time(10,0))
    {
      this->LaunchBall(model, RED);
      this->launchTimes[RED] = this->world->GetSimTime();
    }

    if (model->GetWorldPose().Ign().Pos().Y() >= 10 &&
        (this->world->GetSimTime() - this->launchTimes[BLUE]) >
        common::Time(10, 0))
    {
      this->LaunchBall(model, BLUE);
      this->launchTimes[BLUE] = this->world->GetSimTime();
    }
  }
}

/////////////////////////////////////////////////
void FRC2016GamePlugin::LaunchBall(const physics::ModelPtr _model, Team _side)
{
  std::cout << "Launching[" << _model->GetName() << "]\n";
  _model->SetWorldPose(this->launchPoses[_side]);
}

/////////////////////////////////////////////////
void FRC2016GamePlugin::OnUpdate()
{
  // Get all the models
  physics::Model_V models = this->world->GetModels();

  this->UpdateCastleBalls(models);

  for (auto const &model : models)
  {
    std::string modelName = model->GetName();
    ignition::math::Pose3d modelPose = model->GetWorldPose().Ign();

    // Ignore game pieces.
    if (std::find(this->gamePieces.begin(),
          this->gamePieces.end(), modelName) != this->gamePieces.end())
    {
      continue;
    }

    for (auto &gate : this->gates)
    {
      // Ignore robots that are on the same "team" as the gate.
      if (std::find(this->robots[gate.team].begin(),
                    this->robots[gate.team].end(), modelName) !=
          this->robots[gate.team].end())
      {
        continue;
      }

      if (gate.box.Contains(modelPose.Pos()) && !this->inGate[modelName].active)
      {
        this->inGate[modelName].entered = modelPose;
        this->inGate[modelName].active = true;
      }
      else if (!gate.box.Contains(modelPose.Pos()) &&
               this->inGate[modelName].active)
      {
        this->inGate[modelName].active = false;

        if (this->planes[RED].Side(this->inGate[modelName].entered.Pos()) !=
            this->planes[RED].Side(modelPose.Pos()))
        {
          // todo: check that a blue robot did the scoring
          this->score[BLUE] += gate.crossedCount++ > 0 ? 5 : 3;
          std::cout << "Blue scored on red! Blue score = "
            << this->score[BLUE] << "\n";
        }

        if (this->planes[BLUE].Side(this->inGate[modelName].entered.Pos()) !=
            this->planes[BLUE].Side(modelPose.Pos()))
        {
          // todo: check that a red robot did the scoring
          this->score[RED] += gate.crossedCount++ > 0 ? 5 : 3;
          std::cout << "Red scored on blue! Red score = "
            << this->score[RED] << "\n";
        }
      }
    }
  }
}
