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
#include <string>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/WrenchStamped.h>

// Include Rand.hh first to avoid osx compilation errors
#include "gazebo/math/Rand.hh"
#include "gazebo/rendering/rendering.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
  /// \brief This plugin will generate 5 pictures of a model: perspective,
  /// top, front, side, back.
  class RosBag : public SystemPlugin
  {
    /// \brief Destructor
    public: virtual ~RosBag();

    /// \brief Load the plugin.
    /// \param[in] _argc Number of command line arguments.
    /// \param[in] _argv Array of command line arguments.
    public: void Load(int _argc, char **_argv);

    /// \brief Initialize the plugin.
    private: void Init();

    /// \brief Update the plugin.
    private: void Update(const common::UpdateInfo &_info);

    /// \brief The connections.
    private: event::ConnectionPtr updateConn;

    /// \brief Node for communication.
    private: transport::NodePtr node;

    private: rosbag::Bag bag;
    private: rosbag::View view;
    private: rosbag::View::iterator viewIter;
    private: ros::NodeHandle *rosNode;
    private: ros::Publisher atlasStatePubLeft;
    private: ros::Publisher atlasStatePubRight;

    private: ros::Publisher atlasStatePubUnLeft;
    private: ros::Publisher atlasStatePubUnRight;

    private: ros::Publisher timePub;

    private: double filCoefA[2];
    private: double filCoefB[2];
    private: geometry_msgs::Wrench leftWrenchIn[2];
    private: geometry_msgs::Wrench leftWrenchOut[2];

    private: geometry_msgs::Wrench rightWrenchIn[2];
    private: geometry_msgs::Wrench rightWrenchOut[2];

    private: std::ofstream csvOut;
  };
}
