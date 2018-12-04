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

#include <mutex>

#include <ignition/math/Color.hh>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/transport/Node.hh>
#include "BlinkVisualPlugin.hh"

namespace gazebo
{
  /// \internal
  /// \class BlinkVisualPlugin BlinkVisualPlugin.hh
  /// \brief Private data for the BlinkVisualPlugin class.
  class BlinkVisualPluginPrivate
  {
    /// \brief Visual whose color will be changed.
    public: rendering::VisualPtr visual;

    /// \brief Connects to rendering update event.
    public: event::ConnectionPtr updateConnection;

    /// \brief First color.
    public: ignition::math::Color colorA;

    /// \brief Second color.
    public: ignition::math::Color colorB;

    /// \brief Time taken by a full cycle.
    public: common::Time period;

    /// \brief Time the current cycle started.
    public: common::Time cycleStartTime;

    /// \brief The current simulation time.
    public: common::Time currentSimTime;

    /// \brief Node used for communication.
    public: transport::NodePtr node;

    /// \brief Node used for communication.
    public: std::mutex mutex;

    /// \brief True to use wall time, false to use sim time.
    public: bool useWallTime;

    /// \brief Subscriber to world info.
    public: transport::SubscriberPtr infoSub;
  };
}

using namespace gazebo;

GZ_REGISTER_VISUAL_PLUGIN(BlinkVisualPlugin)

/////////////////////////////////////////////////
BlinkVisualPlugin::BlinkVisualPlugin() : dataPtr(new BlinkVisualPluginPrivate)
{
}

/////////////////////////////////////////////////
BlinkVisualPlugin::~BlinkVisualPlugin()
{
  this->dataPtr->infoSub.reset();
  if (this->dataPtr->node)
    this->dataPtr->node->Fini();
}

/////////////////////////////////////////////////
void BlinkVisualPlugin::Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
{
  if (!_visual || !_sdf)
  {
    gzerr << "No visual or SDF element specified. Plugin won't load." <<
        std::endl;
    return;
  }
  this->dataPtr->visual = _visual;

  // Get color A
  this->dataPtr->colorA.Set(1, 0, 0, 1);
  if (_sdf->HasElement("color_a"))
    this->dataPtr->colorA = _sdf->Get<ignition::math::Color>("color_a");

  // Get color B
  this->dataPtr->colorB.Set(0, 0, 0, 1);
  if (_sdf->HasElement("color_b"))
    this->dataPtr->colorB = _sdf->Get<ignition::math::Color>("color_b");

  // Get the period
  this->dataPtr->period.Set(1);
  if (_sdf->HasElement("period"))
    this->dataPtr->period = _sdf->Get<double>("period");

  if (this->dataPtr->period <= 0)
  {
    gzerr << "Period can't be lower than zero." << std::endl;
    return;
  }

  // Get whether to use wall time or sim time
  this->dataPtr->useWallTime = false;
  if (_sdf->HasElement("use_wall_time"))
    this->dataPtr->useWallTime = _sdf->Get<bool>("use_wall_time");

  // Connect to the world update signal
  this->dataPtr->updateConnection = event::Events::ConnectPreRender(
      std::bind(&BlinkVisualPlugin::Update, this));

  // Subscribe to world statistics to get sim time
  // Warning: topic ~/pose/local/info is meant for high-bandwidth local
  // network access. It will kill the system if a remote gzclient tries to
  // subscribe.
  if (!this->dataPtr->useWallTime)
  {
    this->dataPtr->node = transport::NodePtr(new transport::Node());
    this->dataPtr->node->Init();

    this->dataPtr->infoSub = this->dataPtr->node->Subscribe(
        "~/pose/local/info", &BlinkVisualPlugin::OnInfo, this);
  }
}

/////////////////////////////////////////////////
void BlinkVisualPlugin::Update()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (!this->dataPtr->visual)
  {
    gzerr << "The visual is null." << std::endl;
    return;
  }

  common::Time currentTime;
  if (this->dataPtr->useWallTime)
    currentTime = common::Time::GetWallTime();
  else
    currentTime = this->dataPtr->currentSimTime;

  if (this->dataPtr->cycleStartTime == common::Time::Zero ||
      this->dataPtr->cycleStartTime > currentTime)
  {
    this->dataPtr->cycleStartTime = currentTime;
  }

  auto elapsed = currentTime - this->dataPtr->cycleStartTime;

  // Restart cycle
  if (elapsed >= this->dataPtr->period)
    this->dataPtr->cycleStartTime = currentTime;

  ignition::math::Color from;
  ignition::math::Color to;
  // Color A -> B
  if (elapsed < this->dataPtr->period*0.5)
  {
    from = this->dataPtr->colorA;
    to = this->dataPtr->colorB;
  }
  // Color B -> A
  else
  {
    from = this->dataPtr->colorB;
    to = this->dataPtr->colorA;
    elapsed -= this->dataPtr->period*0.5;
  }

  // interpolate each color component
  double pos = (elapsed/(this->dataPtr->period*0.5)).Double();

  double red = from.R() + (to.R() - from.R()) * pos;
  double green = from.G() + (to.G() - from.G()) * pos;
  double blue = from.B() + (to.B() - from.B()) * pos;
  double alpha = from.A() + (to.A() - from.A()) * pos;

  ignition::math::Color color(red, green, blue, alpha);

  this->dataPtr->visual->SetDiffuse(color);
  this->dataPtr->visual->SetAmbient(color);
  this->dataPtr->visual->SetTransparency(1-color.A());
}

/////////////////////////////////////////////////
void BlinkVisualPlugin::OnInfo(ConstPosesStampedPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->currentSimTime = msgs::Convert(_msg->time());
}
