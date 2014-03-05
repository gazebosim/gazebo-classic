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

#include "gazebo/physics/physics.hh"
#include "plugins/SphereAtlasTestPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(SphereAtlasTestPlugin)

/////////////////////////////////////////////////
SphereAtlasTestPlugin::SphereAtlasTestPlugin()
{
}

/////////////////////////////////////////////////
void SphereAtlasTestPlugin::Load(physics::ModelPtr _model,
                           sdf::ElementPtr /*_sdf*/)
{
  this->model = _model;

  this->jointNames.push_back("l_leg_hpz");
  this->jointNames.push_back("l_leg_hpx");
  this->jointNames.push_back("l_leg_hpy");
  this->jointNames.push_back("l_leg_kny");
  this->jointNames.push_back("l_leg_aky");
  this->jointNames.push_back("l_leg_akx");
  this->jointNames.push_back("r_leg_hpz");
  this->jointNames.push_back("r_leg_hpx");
  this->jointNames.push_back("r_leg_hpy");
  this->jointNames.push_back("r_leg_kny");
  this->jointNames.push_back("r_leg_aky");
  this->jointNames.push_back("r_leg_akx");
  this->jointNames.push_back("l_arm_shy");
  this->jointNames.push_back("l_arm_shx");
  this->jointNames.push_back("l_arm_ely");
  this->jointNames.push_back("l_arm_elx");
  this->jointNames.push_back("l_arm_wry");
  this->jointNames.push_back("l_arm_wrx");
  this->jointNames.push_back("r_arm_shy");
  this->jointNames.push_back("r_arm_shx");
  this->jointNames.push_back("r_arm_ely");
  this->jointNames.push_back("r_arm_elx");
  this->jointNames.push_back("r_arm_wry");
  this->jointNames.push_back("r_arm_wrx");
  this->jointNames.push_back("neck_ry");
  this->jointNames.push_back("back_bkz");
  this->jointNames.push_back("back_bky");
  this->jointNames.push_back("back_bkx");

  this->jointKp.push_back(  1000.0);  // l_leg_hpz        NOLINT
  this->jointKp.push_back(  1000.0);  // l_leg_hpx        NOLINT
  this->jointKp.push_back(  2000.0);  // l_leg_hpy        NOLINT
  this->jointKp.push_back(  5000.0);  // l_leg_kny        NOLINT
  this->jointKp.push_back(  3000.0);  // l_leg_aky        NOLINT
  this->jointKp.push_back(  1000.0);  // l_leg_akx        NOLINT
  this->jointKp.push_back(  1000.0);  // r_leg_hpz        NOLINT
  this->jointKp.push_back(  1000.0);  // r_leg_hpx        NOLINT
  this->jointKp.push_back(  2000.0);  // r_leg_hpy        NOLINT
  this->jointKp.push_back(  5000.0);  // r_leg_kny        NOLINT
  this->jointKp.push_back(  3000.0);  // r_leg_aky        NOLINT
  this->jointKp.push_back(  1000.0);  // r_leg_akx        NOLINT
  this->jointKp.push_back(  2000.0);  // l_arm_shy        NOLINT
  this->jointKp.push_back(  1000.0);  // l_arm_shx        NOLINT
  this->jointKp.push_back(   200.0);  // l_arm_ely        NOLINT
  this->jointKp.push_back(   200.0);  // l_arm_elx        NOLINT
  this->jointKp.push_back(    50.0);  // l_arm_wry        NOLINT
  this->jointKp.push_back(   100.0);  // l_arm_wrx        NOLINT
  this->jointKp.push_back(  2000.0);  // r_arm_shy        NOLINT
  this->jointKp.push_back(  1000.0);  // r_arm_shx        NOLINT
  this->jointKp.push_back(   200.0);  // r_arm_ely        NOLINT
  this->jointKp.push_back(   200.0);  // r_arm_elx        NOLINT
  this->jointKp.push_back(    50.0);  // r_arm_wry        NOLINT
  this->jointKp.push_back(   100.0);  // r_arm_wrx        NOLINT
  this->jointKp.push_back(  1000.0);  // neck_ry          NOLINT
  this->jointKp.push_back( 20000.0);  // back_bkz         NOLINT
  this->jointKp.push_back(200000.0);  // back_bky         NOLINT
  this->jointKp.push_back(200000.0);  // back_bkx         NOLINT

  this->jointKd.push_back( 0.01);  // l_leg_hpz           NOLINT
  this->jointKd.push_back( 1.00);  // l_leg_hpx           NOLINT
  this->jointKd.push_back(10.00);  // l_leg_hpy           NOLINT
  this->jointKd.push_back(10.00);  // l_leg_kny           NOLINT
  this->jointKd.push_back( 2.00);  // l_leg_aky           NOLINT
  this->jointKd.push_back( 1.00);  // l_leg_akx           NOLINT
  this->jointKd.push_back( 0.01);  // r_leg_hpz           NOLINT
  this->jointKd.push_back( 1.00);  // r_leg_hpx           NOLINT
  this->jointKd.push_back(10.00);  // r_leg_hpy           NOLINT
  this->jointKd.push_back(10.00);  // r_leg_kny           NOLINT
  this->jointKd.push_back( 2.00);  // r_leg_aky           NOLINT
  this->jointKd.push_back( 1.00);  // r_leg_akx           NOLINT
  this->jointKd.push_back( 3.00);  // l_arm_shy           NOLINT
  this->jointKd.push_back(10.00);  // l_arm_shx           NOLINT
  this->jointKd.push_back( 3.00);  // l_arm_ely           NOLINT
  this->jointKd.push_back( 3.00);  // l_arm_elx           NOLINT
  this->jointKd.push_back( 0.10);  // l_arm_wry           NOLINT
  this->jointKd.push_back( 0.20);  // l_arm_wrx           NOLINT
  this->jointKd.push_back( 3.00);  // r_arm_shy           NOLINT
  this->jointKd.push_back(10.00);  // r_arm_shx           NOLINT
  this->jointKd.push_back( 3.00);  // r_arm_ely           NOLINT
  this->jointKd.push_back( 3.00);  // r_arm_elx           NOLINT
  this->jointKd.push_back( 0.10);  // r_arm_wry           NOLINT
  this->jointKd.push_back( 0.20);  // r_arm_wrx           NOLINT
  this->jointKd.push_back( 1.00);  // neck_ry             NOLINT
  this->jointKd.push_back( 1.00);  // back_bkz            NOLINT
  this->jointKd.push_back( 2.00);  // back_bky            NOLINT
  this->jointKd.push_back( 1.00);  // back_bkx            NOLINT

  this->qp.push_back(0.0);
  this->qp.push_back(0.0);
  this->qp.push_back(0.0);
  this->qp.push_back(0.0);
  this->qp.push_back(0.0);
  this->qp.push_back(0.0);
  this->qp.push_back(0.0);
  this->qp.push_back(0.0);
  this->qp.push_back(0.0);
  this->qp.push_back(0.0);
  this->qp.push_back(0.0);
  this->qp.push_back(0.0);
  this->qp.push_back(0.0);
  this->qp.push_back(0.0);
  this->qp.push_back(0.0);
  this->qp.push_back(0.0);
  this->qp.push_back(0.0);
  this->qp.push_back(0.0);
  this->qp.push_back(0.0);
  this->qp.push_back(0.0);
  this->qp.push_back(0.0);
  this->qp.push_back(0.0);
  this->qp.push_back(0.0);
  this->qp.push_back(0.0);
  this->qp.push_back(0.0);
  this->qp.push_back(0.0);
  this->qp.push_back(0.0);
  this->qp.push_back(0.0);

  for (unsigned int i = 0; i < this->jointNames.size(); ++i)
  {
    physics::JointPtr j = model->GetJoint(this->jointNames[i]);
    this->joints.push_back(j);
  }

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&SphereAtlasTestPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void SphereAtlasTestPlugin::Init()
{
}

/////////////////////////////////////////////////
void SphereAtlasTestPlugin::Reset()
{
  gzlog << "SphereAtlasTestPlugin: \n"
        << "  This is not a typical usage of plugin Reset function,\n"
        << "  we are doing this just for testing purposes.\n";
  if (this->updateConnection)
  {
    event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
    this->updateConnection.reset();
  }
}

/////////////////////////////////////////////////
void SphereAtlasTestPlugin::OnUpdate()
{
  common::Time currTime = this->model->GetWorld()->GetSimTime();
  common::Time stepTime = currTime - this->prevUpdateTime;
  this->prevUpdateTime = currTime;
  double dt = stepTime.Double();

  for (unsigned int j = 0; j < this->joints.size(); ++j)
  {
    double p = this->joints[j]->GetAngle(0).Radian();
    double target = 0;
    double perror = target - p;
    double derror = (perror - this->qp[j])/dt;
    this->qp[j] = perror;  // save qp
    double force = this->jointKp[j] * perror +
                   this->jointKd[j] * derror;
    this->joints[j]->SetForce(0, force);
  }
}
