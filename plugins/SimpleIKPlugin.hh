/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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

#ifndef _GAZEBO_SIMPLE_IK_PLUGIN_HH_
#define _GAZEBO_SIMPLE_IK_PLUGIN_HH_

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>


#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"

namespace gazebo
{
  class SimpleIKPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: SimpleIKPlugin();

    /// \brief Destructor
    public: virtual ~SimpleIKPlugin();

    /// \brief Load the controller
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    /// \param[in] _info Update information provided by the server.
    private: void Update(const common::UpdateInfo &_info);

    public: virtual void Reset();

    private: physics::WorldPtr world;
    private: physics::ModelPtr model;
    private: physics::LinkPtr goalModel;

    private: boost::mutex update_mutex;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: physics::LinkPtr baseLink;

    private: physics::JointControllerPtr jointController;

    private: math::Vector3 goalPos;

    private: bool useCollisionFrame;

    private: KDL::ChainFkSolverPos_recursive *fkSolver;
    private: KDL::ChainIkSolverVel_wdls *iksolverVel;
    private: KDL::Chain chain;
    private: KDL::JntArray *jointpositions;

    private: physics::Joint_V joints;

    private: common::Time updateRate, lastUpdateTime;
    private: double vels[6];

    private: bool restart;
    private: KDL::ChainIkSolverPos_LMA *iksolverPos;
  };
}
#endif
