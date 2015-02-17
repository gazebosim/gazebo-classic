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
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>

namespace gazebo
{
  class ModelMovePath : public ModelPlugin
  {
  private: void move(math::Vector3 *start, math::Vector3 *end, math::Vector3 *translation, gazebo::common::PoseAnimationPtr anim) {

    int duration = floor(start->Distance((*end).x, (*end).y, (*end).z));
    math::Vector3 diff = *end - *start;
    float x_step = diff.x / duration;
    float y_step = diff.y / duration;
    float z_step = diff.z / duration;
    int curr_frame = anim->GetKeyFrameCount();
    gazebo::common::PoseKeyFrame *key;
    
    for (int i=1; i <= duration; i++) {
      key = anim->CreateKeyFrame(i+curr_frame);
      key->SetTranslation(math::Vector3((*translation).x + x_step*i, 
					(*translation).y + y_step*i, 
					  (*translation).z + z_step*i)); 
      key->SetRotation(math::Quaternion(0, 0, 0));
    }
    
    translation->Set((*translation).x + x_step*duration, 
		     (*translation).y + y_step*duration, 
		     (*translation).z + z_step*duration);
  }

    
  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;
      
      if (!_sdf->HasElement("path"))
	gzerr << "ModelMove plugin missing <path> element\n";

      if (!_sdf->HasElement("n_points"))
	gzerr << "ModelMove plugin missing <n_points> element\n";

      // TODO: Get from sdf
      math::Vector3 start_point = math::Vector3(0, 0, 0);

      const int n_points = std::stoi(_sdf->GetElement("n_points")->Get<std::string>());

      math::Vector3 *path = new math::Vector3[n_points];
      std::stringstream stream(_sdf->GetElement("path")->Get<std::string>());
      int i = 0;
      while(i < n_points) {
	float f1, f2, f3;
	stream >> f1 >> f2 >> f3;
        path[i] = math::Vector3(f1, f2, f3);
	i++;
	}

      float path_length = start_point.Distance(path->x, path->y, path->z);
      for (int i=0; i < n_points-1; i++) {
	path_length += path[i].Distance(path[i+1].x, path[i+1].y, path[i+1].z);
      }	

      // create the animation
      gazebo::common::PoseAnimationPtr anim(
					    // name the animation "test",
					    // make it last 10 seconds,
					    // and set it on a repeat loop
					    new gazebo::common::PoseAnimation("test", path_length+1, false));
      
      gazebo::common::PoseKeyFrame *key;
      
      // set starting location of the box
      key = anim->CreateKeyFrame(0);
      key->SetTranslation(math::Vector3(0, 0, 0));
      key->SetRotation(math::Quaternion(0, 0, 0));

      math::Vector3 translation = math::Vector3(0, 0, 0);

      move(&start_point, path, &translation, anim); 
      for (int i=0; i < n_points-1; i++) {
	move(path+i, path+i+1, &translation, anim);
      }	
      
      // set the animation
      _parent->SetAnimation(anim);
    }
    
    // Pointer to the model
  private: physics::ModelPtr model;

    // Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelMovePath);
}
