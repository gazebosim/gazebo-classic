/*
 * Copyright 2012 Open Source Robotics Foundation
 * Copyright 2013 Dereck Wonnacott
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
#include "plugins/SkidSteerDrivePlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(SkidSteerDrivePlugin)


/////////////////////////////////////////////////
SkidSteerDrivePlugin::SkidSteerDrivePlugin()
{
  this->MaxForce = 5.0;
  this->wheelRadius = 0.0;
  this->wheelSeparation = 0.0;
}

/////////////////////////////////////////////////
int SkidSteerDrivePlugin::RegisterJoint(int index, std::string name)
{  
  this->Joints[index] = this->model->GetJoint(name);
  
  if (this->Joints[index]) return 0;
  
  gzerr << "Unable to find the " << name 
        <<  " joint in model " << this->model->GetName() << "." << std::endl;
  
  return 1;
}

/////////////////////////////////////////////////
void SkidSteerDrivePlugin::Load(physics::ModelPtr _model,
                           sdf::ElementPtr _sdf)
{
  this->model = _model;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());
  
  int err = 0;
  err += RegisterJoint(RIGHT_FRONT, "right_front");
  err += RegisterJoint(RIGHT_REAR,  "right_rear");
  err += RegisterJoint(LEFT_FRONT,  "left_front");
  err += RegisterJoint(LEFT_REAR,   "left_rear");
  if(err > 0)  return;


  if (_sdf->HasElement("MaxForce"))
    this->MaxForce = _sdf->GetElement("MaxForce")->GetValueDouble();
  else
    gzwarn << "No MaxForce value set in the model sdf, default value is 5.0.\n";
  
  
  // This assumes that front and rear wheel spacing is identical
  this->wheelSeparation = this->Joints[RIGHT_FRONT]->GetAnchor(0).Distance(
                          this->Joints[LEFT_FRONT]->GetAnchor(0));                
                      
  // This assumes that the largest dimension of the wheel is the diameter
  // and that all wheels have the same diameter
  physics::EntityPtr wheel_link = boost::shared_dynamic_cast<physics::Entity>(this->Joints[RIGHT_FRONT]->GetChild() );
  if(wheel_link)
  {
    math::Box bb = wheel_link->GetBoundingBox();
    this->wheelRadius = bb.GetSize().GetMax() * 0.5;
  }
  
  // Stupid checks...
  if (this->wheelSeparation <= 0)
  {
    gzerr << "Unable to find the wheel separation distance." << std::endl
          << "  This could mean that the right_front link and the left_front link are overlapping." << std::endl;
    return;
  }   
  if (this->wheelRadius <= 0)
  {
    gzerr << "Unable to find the wheel radius." << std::endl
          << "  This could mean that the sdf is missing a wheel link on the right_front joint." << std::endl;
    return;
  }
        
  this->velSub = this->node->Subscribe( 
    std::string("~/") + this->model->GetName() + std::string("/vel_cmd"), 
    &SkidSteerDrivePlugin::OnVelMsg, this);
}


/////////////////////////////////////////////////
void SkidSteerDrivePlugin::OnVelMsg(ConstPosePtr &msg)
{
  //gzmsg << "cmd_vel: " << msg->position().x()  
  //      << ", "        << msgs::Convert(msg->orientation()).GetAsEuler().z << std::endl;

  for(int i = 0; i < NUMBER_OF_WHEELS; i++)
    this->Joints[i]->SetMaxForce(0, this->MaxForce);

  double vel_lin = msg->position().x() / this->wheelRadius;
  double vel_rot = -1 * msgs::Convert(msg->orientation()).GetAsEuler().z 
                   * (this->wheelSeparation / this->wheelRadius);

  this->Joints[RIGHT_FRONT]->SetVelocity(0, vel_lin - vel_rot);
  this->Joints[RIGHT_REAR ]->SetVelocity(0, vel_lin - vel_rot); 
  this->Joints[LEFT_FRONT ]->SetVelocity(0, vel_lin + vel_rot); 
  this->Joints[LEFT_REAR  ]->SetVelocity(0, vel_lin + vel_rot);
}



