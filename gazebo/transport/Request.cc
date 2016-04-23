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

#include <ignition/transport/Node.hh>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/Request.hh"

using namespace gazebo;

/////////////////////////////////////////////////
void transport::RequestEntityDelete(const std::string &_name)
{
  // Unused callback
  std::function<void(const msgs::Empty &, const bool)> unused =
    [](const msgs::Empty &, const bool &)
  {
  };

  // Operation msg
  msgs::Operation req;
  req.set_type(msgs::Operation::DELETE_ENTITY);
  req.set_uri(_name);

  // Request
  ignition::transport::Node ignNode;
  ignNode.Request("/request", req, unused);
}

/////////////////////////////////////////////////
void transport::RequestEntityInsert(const std::string &_sdf,
    const ignition::math::Pose3d &_pose)
{
  // Unused callback
  std::function<void(const msgs::Empty &, const bool)> unused =
    [](const msgs::Empty &, const bool &)
  {
  };

  // Factory msg
  msgs::Factory msg;
  msg.set_sdf(_sdf);
  if (_pose != ignition::math::Pose3d(IGN_DBL_MAX, IGN_DBL_MAX, IGN_DBL_MAX,
                                      0, 0, 0))
  {
    msgs::Set(msg.mutable_pose(), _pose);
  }

  // Operation msg
  msgs::Operation req;
  req.set_type(msgs::Operation::INSERT_ENTITY);
  req.mutable_factory()->CopyFrom(msg);

  // Request
  ignition::transport::Node ignNode;
  ignNode.Request("/request", req, unused);
}

/////////////////////////////////////////////////
void transport::RequestEntityClone(const std::string &_name,
    const ignition::math::Pose3d &_pose)
{
  // Unused callback
  std::function<void(const msgs::Empty &, const bool)> unused =
    [](const msgs::Empty &, const bool &)
  {
  };

  // Factory msg
  msgs::Factory msg;
  msg.set_clone_model_name(_name);
  if (_pose != ignition::math::Pose3d(IGN_DBL_MAX, IGN_DBL_MAX, IGN_DBL_MAX,
                                      0, 0, 0))
  {
    msgs::Set(msg.mutable_pose(), _pose);
  }

  // Operation msg
  msgs::Operation req;
  req.set_type(msgs::Operation::INSERT_ENTITY);
  req.mutable_factory()->CopyFrom(msg);

  // Request
  ignition::transport::Node ignNode;
  ignNode.Request("/request", req, unused);
}

