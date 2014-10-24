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

#ifndef _GAZEBO_ARRANGE_PLUGIN_HH_
#define _GAZEBO_ARRANGE_PLUGIN_HH_

#include <map>
#include <string>

#include <boost/shared_ptr.hpp>

#include <sdf/sdf.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/util/system.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  class GAZEBO_VISIBLE ArrangePlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: ArrangePlugin();

    /// \brief Destructor.
    public: ~ArrangePlugin();

    /// \brief Load the plugin.
    /// \param[in] _world Pointer to world
    /// \param[in] _sdf Pointer to the SDF configuration.
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Initialize the plugin.
    public: virtual void Init();

    /// \brief Reset the plugin.
    public: virtual void Reset();

    /// \brief Set up the arrangement with name given by input parameter.
    /// \param[in] _arrangement Name of arrangement.
    /// \return True if arrangement was set successfully.
    public: bool SetArrangement(const std::string &_arrangement);

    /// \brief Callback function which receives a requested arrangement name.
    public: void ArrangementCallback(ConstGzStringPtr &_msg);

    /// \brief World pointer.
    protected: physics::WorldPtr world;

    /// \brief SDF pointer.
    protected: sdf::ElementPtr sdf;

    /// \brief Class to store info about each object.
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

    /// \brief Collection of models.
    protected: Object_M objects;

    /// \brief Map of strings to model poses.
    typedef std::map<std::string, math::Pose> Pose_M;

    /// \brief Map of strings to Pose_M (arrangement map).
    typedef std::map<std::string, Pose_M> Arrangement_M;

    /// \brief Information about arrangements.
    protected: Arrangement_M arrangements;

    /// \brief Initial arrangement name.
    protected: std::string initialArrangementName;

    /// \brief Current arrangement name.
    protected: std::string currentArrangementName;

    /// \brief Topic to listen on for changing arrangments.
    protected: std::string eventTopicName;

    /// \brief Node for Gazebo transport.
    protected: transport::NodePtr node;

    /// \brief Subscriber for listening to changing arrangements.
    protected: transport::SubscriberPtr sub;
  };
}
#endif
