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

#include <functional>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector2.hh>

#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/rendering/Conversions.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/selection_buffer/SelectionBuffer.hh>

#include "plugins/FiducialCameraPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(FiducialCameraPlugin)

namespace gazebo
{
  class FiducialCameraPluginPrivate
  {
    /// \brief Pointer to the parent camera sensor
    public: sensors::CameraSensorPtr parentSensor;

    /// \brief Selection buffer used for occlusion detection
    public: rendering::SelectionBuffer *selectionBuffer = nullptr;

    /// \brief All event connections.
    public: std::vector<event::ConnectionPtr> connections;

    /// \brief A list of fiducials tracked by this camera.
    public: std::vector<std::string> fiducials;

    /// \brief Transport node used for publishing fiducial messages.
    public: transport::NodePtr node;

    /// \brief Publisher of fiducial messages.
    public: transport::PublisherPtr fiducialPub;
  };
}

/////////////////////////////////////////////////
ignition::math::Vector2i GetScreenSpaceCoords(ignition::math::Vector3d _pt,
    gazebo::rendering::CameraPtr _cam)
{
  // Convert from 3D world pos to 2D screen pos
  Ogre::Vector3 pos = _cam->OgreCamera()->getProjectionMatrix() *
      _cam->OgreCamera()->getViewMatrix() *
      gazebo::rendering::Conversions::Convert(_pt);

  ignition::math::Vector2i screenPos;
  screenPos.X() = ((pos.x / 2.0) + 0.5) * _cam->ViewportWidth();
  screenPos.Y() = (1 - ((pos.y / 2.0) + 0.5)) * _cam->ViewportHeight();

  return screenPos;
}

/////////////////////////////////////////////////
FiducialCameraPlugin::FiducialCameraPlugin()
    : SensorPlugin(),
      dataPtr(new FiducialCameraPluginPrivate)
{
}

/////////////////////////////////////////////////
FiducialCameraPlugin::~FiducialCameraPlugin()
{
  delete this->dataPtr->selectionBuffer;

  this->dataPtr->connections.clear();
  this->dataPtr->parentSensor.reset();
}

/////////////////////////////////////////////////
void FiducialCameraPlugin::Load(sensors::SensorPtr _sensor,
                                sdf::ElementPtr _sdf)
{
  this->dataPtr->parentSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);

  if (!this->dataPtr->parentSensor)
  {
    gzerr << "FiducialCameraPlugin not attached to a camera sensor\n";
    return;
  }

  // load the fiducials
  if (_sdf->HasElement("fiducial"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("fiducial");
    while (elem)
    {
      this->dataPtr->fiducials.push_back(elem->Get<std::string>());
      elem = elem->GetNextElement("fiducial");
    }
  }
  else
  {
    gzerr << "No fidicuals specified. FiducialCameraPlugin will not be run."
        << std::endl;
    return;
  }

  this->dataPtr->parentSensor->SetActive(true);

  this->dataPtr->connections.push_back(
      this->dataPtr->parentSensor->Camera()->ConnectNewImageFrame(
      std::bind(&FiducialCameraPlugin::OnNewFrame, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5)));
}

/////////////////////////////////////////////////
void FiducialCameraPlugin::Init()
{
  this->dataPtr->node.reset(new transport::Node());
  this->dataPtr->node->Init();

  // Create publisher for tactile messages
  std::string topicName = "~/" + this->dataPtr->parentSensor->ParentName()
      + "/" + this->dataPtr->parentSensor->Name() + "/fiducial";
  boost::replace_all(topicName, "::", "/");
  this->dataPtr->fiducialPub =
      this->dataPtr->node->Advertise<msgs::PosesStamped>(topicName);
}

/////////////////////////////////////////////////
void FiducialCameraPlugin::OnNewFrame(const unsigned char * /*_image*/,
    unsigned int /*_width*/, unsigned int /*_height*/, unsigned int /*_depth*/,
    const std::string &/*_format*/)
{
  rendering::CameraPtr camera = this->dataPtr->parentSensor->Camera();
  rendering::ScenePtr scene = camera->GetScene();

  if (!this->dataPtr->selectionBuffer)
  {
    std::string cameraName = camera->OgreCamera()->getName();
    this->dataPtr->selectionBuffer = new rendering::SelectionBuffer(cameraName,
        scene->OgreSceneManager(),
        camera->RenderTexture()->getBuffer()->getRenderTarget());
  }

  std::vector<FiducialData> results;
  for (const auto &f : this->dataPtr->fiducials)
  {
    // check if fiducial is visible within the frustum
    rendering::VisualPtr vis = scene->GetVisual(f);
    if (!vis)
      continue;

    if (!camera->IsVisible(vis))
      continue;

    ignition::math::Vector2i pt = GetScreenSpaceCoords(
        vis->GetWorldPose().pos.Ign(), camera);

    // use selection buffer to check if visual is occluded by other entities
    // in the camera view
    Ogre::Entity *entity =
      this->dataPtr->selectionBuffer->OnSelectionClick(pt.X(), pt.Y());

    rendering::VisualPtr result;
    if (entity && !entity->getUserObjectBindings().getUserAny().isEmpty())
    {
      try
      {
        result = scene->GetVisual(
            Ogre::any_cast<std::string>(
            entity->getUserObjectBindings().getUserAny()));
      }
      catch(Ogre::Exception &_e)
      {
        gzerr << "Ogre Error:" << _e.getFullDescription() << "\n";
        continue;
      }
    }

    if (result && result->GetRootVisual() == vis)
    {
      FiducialData fd;
      fd.id = vis->GetName();
      fd.pt = pt;
      results.push_back(fd);
    }
  }

  this->Publish(results);
}

/////////////////////////////////////////////////
void FiducialCameraPlugin::Publish(const std::vector<FiducialData> &_results)
{
  // publish the results
  common::Time timestamp = this->dataPtr->parentSensor->LastMeasurementTime();

  msgs::PosesStamped msg;
  msgs::Set(msg.mutable_time(), timestamp);

  for (const auto &fd : _results)
  {
    msgs::Pose *poseMsg = msg.add_pose();
    poseMsg->set_name(fd.id);
    ignition::math::Vector3d pos(fd.pt.X(), fd.pt.Y(), 0);
    msgs::Set(poseMsg, ignition::math::Pose3d(pos,
        ignition::math::Quaterniond::Identity));
  }

  this->dataPtr->fiducialPub->Publish(msg);
}
