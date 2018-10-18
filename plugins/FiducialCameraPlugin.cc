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

#include <vector>
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
  /// \brief A class to store fiducial data
  class FiducialData
  {
    /// \brief Fiducial ID
    public: std::string id;

    /// \brief Center point of the fiducial in the image
    public: ignition::math::Vector2i pt;
  };

  class FiducialCameraPluginPrivate
  {
    /// \brief Pointer to the parent camera sensor
    public: sensors::CameraSensorPtr parentSensor;

    /// \brief Selection buffer used for occlusion detection
    public: std::unique_ptr<rendering::SelectionBuffer> selectionBuffer;

    /// \brief All event connections.
    public: std::vector<event::ConnectionPtr> connections;

    /// \brief A list of fiducials tracked by this camera.
    public: std::set<std::string> fiducials;

    /// \brief Transport node used for publishing fiducial messages.
    public: transport::NodePtr node;

    /// \brief Publisher of fiducial messages.
    public: transport::PublisherPtr fiducialPub;

    /// \brief True to detect all objects in the world.
    public: bool detectAll = false;

    /// \brief Pointer to the camera.
    public: rendering::CameraPtr camera;

    /// \brief Pointer to the scene.
    public: rendering::ScenePtr scene;

    /// \brief Publish the results
    /// \param[in] _results Fiducial data containing id and location in image.
    public: void Publish(const std::vector<FiducialData> &_results) const;
  };
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
  this->dataPtr->fiducialPub.reset();
  this->dataPtr->node->Fini();

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
      this->dataPtr->fiducials.insert(elem->Get<std::string>());
      elem = elem->GetNextElement("fiducial");
    }
  }
  else
  {
    gzmsg << "No fiducials specified. All models will be tracked."
        << std::endl;
    this->dataPtr->detectAll = true;
  }

  this->dataPtr->parentSensor->SetActive(true);

  this->dataPtr->camera = this->dataPtr->parentSensor->Camera();
  if (this->dataPtr->camera)
  {
    this->dataPtr->scene = this->dataPtr->camera->GetScene();
    if (this->dataPtr->scene)
    {
      this->dataPtr->connections.push_back(
          this->dataPtr->parentSensor->Camera()->ConnectNewImageFrame(
          std::bind(&FiducialCameraPlugin::OnNewFrame, this,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
            std::placeholders::_4, std::placeholders::_5)));
    }
  }
  if (!this->dataPtr->camera || !this->dataPtr->scene)
  {
    gzerr << "FiducialCameraPlugin failed to load. "
        << "Camera and/or Scene not found" << std::endl;
  }
}

/////////////////////////////////////////////////
void FiducialCameraPlugin::Init()
{
  this->dataPtr->node.reset(new transport::Node());
  this->dataPtr->node->Init();

  // Create publisher for fiducial messages
  std::string topicName = "~/" + this->dataPtr->parentSensor->ParentName()
      + "/" + this->dataPtr->parentSensor->Name() + "/fiducial";

  size_t pos;
  while ((pos = topicName.find("::")) != std::string::npos)
    topicName = topicName.substr(0, pos) + "/" + topicName.substr(pos+2);

  this->dataPtr->fiducialPub =
      this->dataPtr->node->Advertise<msgs::PosesStamped>(topicName);
}

/////////////////////////////////////////////////
void FiducialCameraPlugin::PopulateFiducials()
{
  this->dataPtr->fiducials.clear();

  // Check all models for inclusion in the frustum.
  rendering::VisualPtr worldVis = this->dataPtr->scene->WorldVisual();
  for (unsigned int i = 0; i < worldVis->GetChildCount(); ++i)
  {
    rendering::VisualPtr childVis = worldVis->GetChild(i);
    if (childVis->GetType() == rendering::Visual::VT_MODEL)
      this->dataPtr->fiducials.insert(childVis->GetName());
  }
}

/////////////////////////////////////////////////
void FiducialCameraPlugin::OnNewFrame(const unsigned char */*_image*/,
    const unsigned int /*_width*/, const unsigned int /*_height*/,
    const unsigned int /*_depth*/, const std::string &/*_format*/)
{
  if (!this->dataPtr->selectionBuffer)
  {
    std::string cameraName = this->dataPtr->camera->OgreCamera()->getName();
    this->dataPtr->selectionBuffer.reset(
        new rendering::SelectionBuffer(cameraName,
        this->dataPtr->scene->OgreSceneManager(),
        this->dataPtr->camera->RenderTexture()->getBuffer()->
        getRenderTarget()));
  }

  if (this->dataPtr->detectAll)
    this->PopulateFiducials();

  std::vector<FiducialData> results;
  for (const auto &f : this->dataPtr->fiducials)
  {
    // check if fiducial is visible within the frustum
    rendering::VisualPtr vis = this->dataPtr->scene->GetVisual(f);
    if (!vis)
      continue;

    if (!this->dataPtr->camera->IsVisible(vis))
      continue;

    ignition::math::Vector2i pt =
        this->dataPtr->camera->Project(vis->GetWorldPose().pos.Ign());

    // use selection buffer to check if visual is occluded by other entities
    // in the camera view
    Ogre::Entity *entity =
      this->dataPtr->selectionBuffer->OnSelectionClick(pt.X(), pt.Y());

    rendering::VisualPtr result;
    if (entity && !entity->getUserObjectBindings().getUserAny().isEmpty())
    {
      try
      {
        result = this->dataPtr->scene->GetVisual(
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

  this->dataPtr->Publish(results);
}

/////////////////////////////////////////////////
void FiducialCameraPluginPrivate::Publish(
    const std::vector<FiducialData> &_results) const
{
  // publish the results
  common::Time timestamp = this->parentSensor->LastMeasurementTime();

  msgs::PosesStamped msg;
  msgs::Set(msg.mutable_time(), timestamp);

  for (const auto &fd : _results)
  {
    // use pose msg to store the result
    // position x and y are image coordinates and z always 0
    // orientation is always an identity quaternion for now
    msgs::Pose *poseMsg = msg.add_pose();
    poseMsg->set_name(fd.id);
    ignition::math::Vector3d pos(fd.pt.X(), fd.pt.Y(), 0);
    msgs::Set(poseMsg, ignition::math::Pose3d(pos,
        ignition::math::Quaterniond::Identity));
  }

  this->fiducialPub->Publish(msg);
}
