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

#ifndef _GAZEBO_ARAT_PLUGIN_HH_
#define _GAZEBO_ARAT_PLUGIN_HH_

#include <map>
#include <string>

#include <boost/shared_ptr.hpp>

#include <sdf/sdf.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include "gazebo/util/system.hh"

namespace gazebo
{
  class GAZEBO_VISIBLE ARATPlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: ARATPlugin();

    /// \brief Load the plugin.
    /// \param[in] _world Pointer to world
    /// \param[in] _sdf Pointer to the SDF configuration.
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Initialize the plugin.
    public: virtual void Init();

    /// \brief Set up the task with name given by input parameter.
    /// \param[in] _task Name of task.
    /// \return True if task was set successfully.
    public: bool SetTask(const std::string &_task);

    /// \brief World pointer.
    protected: physics::WorldPtr world;

    /// \brief SDF pointer.
    protected: sdf::ElementPtr sdf;

    /// \brief Class to store info about each ARAT object.
    protected: class Object
               {
                 /// \brief Model pointer.
                 public: physics::ModelPtr model;

                 /// \brief Initial object pose.
                 public: math::Pose pose;
               };
    typedef boost::shared_ptr<Object> ObjectPtr;

    /// \brief Map of strings to model pointers.
    typedef std::map<std::string, ObjectPtr> Object_M;

    /// \brief Collection of ARAT models.
    protected: Object_M objects;

    /// \brief Map of strings to model poses.
    typedef std::map<std::string, math::Pose> Pose_M;

    /// \brief Map of strings to Pose_M (task map).
    typedef std::map<std::string, Pose_M> Task_M;

    /// \brief Information about tasks.
    protected: Task_M tasks;
  };
}
#endif
