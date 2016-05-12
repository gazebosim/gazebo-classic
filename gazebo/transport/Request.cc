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

#include <ignition/math/Rand.hh>
#include <ignition/transport/Node.hh>

#include "gazebo/common/Console.hh"
#include "gazebo/transport/Request.hh"

using namespace gazebo;

// Unused callback
std::function<void(const ignition::msgs::Empty &, const bool)> unused =
  [](const ignition::msgs::Empty &, const bool &)
{
};

/////////////////////////////////////////////////
size_t transport::RequestDelete(const std::string &_name)
{
  // Unique id for request
  auto id = ignition::math::Rand::IntUniform(1, 10000);

  // Operation msg
  ignition::msgs::Operation req;
  req.set_type(ignition::msgs::Operation::DELETE_ENTITY);
  req.set_id(id);
  req.set_uri(_name);

  // Request
  ignition::transport::Node ignNode;
  ignNode.Request("/request", req, unused);

  return id;
}

/////////////////////////////////////////////////
size_t transport::RequestInsertSDF(const std::string &_sdf,
    const ignition::math::Pose3d &_pose)
{
  // Unique id for request
  auto id = ignition::math::Rand::IntUniform(1, 10000);

  // EntityFactory msg
  ignition::msgs::EntityFactory msg;
  msg.set_sdf(_sdf);
  if (_pose != ignition::math::Pose3d(IGN_DBL_MAX, IGN_DBL_MAX, IGN_DBL_MAX,
                                      0, 0, 0))
  {
    ignition::msgs::Set(msg.mutable_pose(), _pose);
  }

  // Operation msg
  ignition::msgs::Operation req;
  req.set_type(ignition::msgs::Operation::INSERT_ENTITY);
  req.set_id(id);
  req.mutable_factory()->CopyFrom(msg);

  // Request
  ignition::transport::Node ignNode;
  ignNode.Request("/request", req, unused);

  return id;
}

/////////////////////////////////////////////////
size_t transport::RequestInsertFile(const std::string &_filename,
    const ignition::math::Pose3d &_pose)
{
  // Unique id for request
  auto id = ignition::math::Rand::IntUniform(1, 10000);

  // EntityFactory msg
  ignition::msgs::EntityFactory msg;
  msg.set_sdf_filename(_filename);
  if (_pose != ignition::math::Pose3d(IGN_DBL_MAX, IGN_DBL_MAX, IGN_DBL_MAX,
                                      0, 0, 0))
  {
    ignition::msgs::Set(msg.mutable_pose(), _pose);
  }

  // Operation msg
  ignition::msgs::Operation req;
  req.set_type(ignition::msgs::Operation::INSERT_ENTITY);
  req.set_id(id);
  req.mutable_factory()->CopyFrom(msg);

  // Request
  ignition::transport::Node ignNode;
  ignNode.Request("/request", req, unused);

  return id;
}

/////////////////////////////////////////////////
size_t transport::RequestClone(const std::string &_name,
    const ignition::math::Pose3d &_pose)
{
  // Unique id for request
  auto id = ignition::math::Rand::IntUniform(1, 10000);

  // EntityFactory msg
  ignition::msgs::EntityFactory msg;
  msg.set_clone_model_name(_name);
  if (_pose != ignition::math::Pose3d(IGN_DBL_MAX, IGN_DBL_MAX, IGN_DBL_MAX,
                                      0, 0, 0))
  {
    ignition::msgs::Set(msg.mutable_pose(), _pose);
  }

  // Operation msg
  ignition::msgs::Operation req;
  req.set_type(ignition::msgs::Operation::INSERT_ENTITY);
  req.set_id(id);
  req.mutable_factory()->CopyFrom(msg);

  // Request
  ignition::transport::Node ignNode;
  ignNode.Request("/request", req, unused);

  return id;
}

/////////////////////////////////////////////////
size_t transport::RequestInsert(const ignition::msgs::Light &_msg)
{
  // Unique id for request
  auto id = ignition::math::Rand::IntUniform(1, 10000);

  // Operation msg
  ignition::msgs::EntityFactory fac;
  fac.mutable_light()->CopyFrom(_msg);

  ignition::msgs::Operation req;
  req.set_type(ignition::msgs::Operation::INSERT_LIGHT);
  req.set_id(id);
  req.mutable_factory()->CopyFrom(fac);

  // Request
  ignition::transport::Node ignNode;
  ignNode.Request("/request", req, unused);

  return id;
}

/////////////////////////////////////////////////
size_t transport::RequestInsert(const ignition::msgs::EntityFactory &_msg)
{
  // Unique id for request
  auto id = ignition::math::Rand::IntUniform(1, 10000);

  // Operation msg
  ignition::msgs::Operation req;
  req.set_type(ignition::msgs::Operation::INSERT_LIGHT);
  req.set_id(id);
  req.mutable_factory()->CopyFrom(_msg);

  // Request
  ignition::transport::Node ignNode;
  ignNode.Request("/request", req, unused);

  return id;
}

