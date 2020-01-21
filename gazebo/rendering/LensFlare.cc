/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include "gazebo/common/Assert.hh"

#include "gazebo/transport/Node.hh"

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Light.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/LensFlare.hh"
#include "gazebo/rendering/WideAngleCamera.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \brief We'll create an instance of this class for each camera, to be
    /// used to inject lens flare uniforms and time (for animating flare)
    /// in each render call.
    class LensFlareCompositorListener
      : public Ogre::CompositorInstance::Listener
    {
      /// \brief Constructor
      public: LensFlareCompositorListener(CameraPtr _camera, LightPtr _light)
      {
        this->camera = _camera;
        this->SetLight(_light);
      }

      /// \brief Destructor
      public: ~LensFlareCompositorListener()
      {
        if (this->wideAngleDummyCamera)
        {
          this->wideAngleDummyCamera->GetScene()->RemoveCamera(
              this->wideAngleDummyCamera->Name());
        }
      }

      /// \brief Set directional light that generates lens flare
      /// \param[in] _light Pointer to directional light
      public: void SetLight(LightPtr _light)
      {
        this->light = _light;
      }

      /// \brief Set the scale of lens flare.
      /// \param[in] _scale Scale of lens flare
      public: void SetScale(const double _scale)
      {
        this->scale = _scale;
      }

      /// \brief Callback that OGRE will invoke for us on each render call
      /// \param[in] _passID OGRE material pass ID.
      /// \param[in] _mat Pointer to OGRE material.
      public: virtual void notifyMaterialRender(unsigned int _passId,
                                                Ogre::MaterialPtr &_mat)
      {
        GZ_ASSERT(!_mat.isNull(), "Null OGRE material");
        // These calls are setting parameters that are declared in two places:
        // 1. media/materials/scripts/gazebo.material, in
        //    fragment_program Gazebo/CameraLensFlareFS
        // 2. media/materials/scripts/camera_lens_flare_fs.glsl
        Ogre::Technique *technique = _mat->getTechnique(0);
        GZ_ASSERT(technique, "Null OGRE material technique");
        Ogre::Pass *pass = technique->getPass(_passId);
        GZ_ASSERT(pass, "Null OGRE material pass");
        Ogre::GpuProgramParametersSharedPtr params =
            pass->getFragmentProgramParameters();
        GZ_ASSERT(!params.isNull(), "Null OGRE material GPU parameters");

        // used for animating flare
        params->setNamedConstant("time", static_cast<Ogre::Real>(
            common::Time::GetWallTime().Double()));
        // for adjusting aspect ratio of flare
        params->setNamedConstant("viewport",
            Ogre::Vector3(static_cast<double>(this->camera->ViewportWidth()),
            static_cast<double>(this->camera->ViewportHeight()), 1.0));

        // use light's world position for lens flare position
        if (this->light->Type() == "directional")
        {
          // Directional lights misuse position as a direction.
          // The large multiplier is for occlusion testing and assumes the light
          // is very far away. Larger values cause the light to disappear on
          // some frames for some unknown reason.
          this->lightWorldPos = -(this->light->WorldPose().Rot() *
                                  light->Direction()) * 100000.0;
        }
        else
          this->lightWorldPos = this->light->WorldPose().Pos();

        ignition::math::Vector3d pos;
        double lensFlareScale = 1.0;

        // wide angle camera has a different way of projecting 3d points and
        // occlusion checking
        auto wideAngleCam =
            boost::dynamic_pointer_cast<WideAngleCamera>(this->camera);
        if (wideAngleCam)
          this->WideAngleCameraPosScale(wideAngleCam, pos, lensFlareScale);
        else
          this->CameraPosScale(this->camera, pos, lensFlareScale);

        params->setNamedConstant("lightPos", Conversions::Convert(pos));
        params->setNamedConstant("scale",
            static_cast<Ogre::Real>(lensFlareScale));
      }

      /// \brief Get the lens flare position and scale for a normal camera
      /// \param[in] _camera Camera which the lens flare is added to
      /// \param[out] _pos lens flare position in normalized device coordinates
      /// \param[out] _scale Amount to scale the lens flare by.
      private: void CameraPosScale(const CameraPtr &_camera,
          ignition::math::Vector3d &_pos, double &_scale)
      {
        Ogre::Vector3 lightPos;
        // project 3d world space to clip space
        auto viewProj = _camera->OgreCamera()->getProjectionMatrix() *
          _camera->OgreCamera()->getViewMatrix();
        auto pos = viewProj * Ogre::Vector4(
            Conversions::Convert(this->lightWorldPos));
        // normalize x and y
        // keep z for visibility test
        lightPos.x = pos.x / pos.w;
        lightPos.y = pos.y / pos.w;
        lightPos.z = pos.z;

        double occlusionScale = 1.0;
        if (lightPos.z >= 0.0)
        {
          occlusionScale = this->OcclusionScale(_camera,
              Conversions::ConvertIgn(lightPos), this->lightWorldPos);
        }
        _pos = Conversions::ConvertIgn(lightPos);
        _scale = occlusionScale * this->scale;
      }

      /// \brief Get the lens flare position and scale for a wide angle camera
      /// \param[in] _wideAngleCam Camera which the lens flare is added to
      /// \param[out] _pos lens flare position in normalized device coordinates
      /// \param[out] _scale Amount to scale the lens flare by.
      private: void WideAngleCameraPosScale(
          const WideAngleCameraPtr &_wideAngleCam,
          ignition::math::Vector3d &_pos, double &_scale)
      {
        Ogre::Vector3 lightPos;
        // create dummy camera for occlusion checking
        // Needed so we can reuse Scene::FirstContact function which expects
        // a gazebo camera object
        std::vector<Ogre::Camera *> ogreEnvCameras =
            _wideAngleCam->OgreEnvCameras();
        if (!this->wideAngleDummyCamera)
        {
          // create camera with auto render set to false
          // so it doesn't actually use up too much gpu resources
          static unsigned int dummyCamId = 0;
          std::string dummyCamName =
              _wideAngleCam->Name() + "_lensflare_occlusion_" +
              std::to_string(dummyCamId);
          this->wideAngleDummyCamera =
              _wideAngleCam->GetScene()->CreateCamera(
              dummyCamName, false);
          this->wideAngleDummyCamera->Load();

          // set dummy camera properties based on env cam
          Ogre::Camera *cam = ogreEnvCameras[0];
          this->wideAngleDummyCamera->SetImageWidth(
              cam->getViewport()->getActualWidth());
          this->wideAngleDummyCamera->SetImageHeight(
              cam->getViewport()->getActualHeight());
          this->wideAngleDummyCamera->Init();
          this->wideAngleDummyCamera->CreateRenderTexture(
              dummyCamName + "_rtt");
          this->wideAngleDummyCamera->SetAspectRatio(
              cam->getAspectRatio());
          // aspect ratio should be 1.0 so VFOV should equal to HFOV
          this->wideAngleDummyCamera->SetHFOV(
              ignition::math::Angle(cam->getFOVy().valueRadians()));
          // reset camera orientation so we can set the exact world pose
          // below when doing occlusion ray cast test
          this->wideAngleDummyCamera->OgreCamera()->setOrientation(
              Ogre::Quaternion::IDENTITY);
        }

        // project camera into screen space
        double viewportWidth =
            static_cast<double>(_wideAngleCam->ViewportWidth());
        double viewportHeight =
            static_cast<double>(_wideAngleCam->ViewportHeight());
        auto imagePos = _wideAngleCam->Project3d(this->lightWorldPos);

        GZ_ASSERT(viewportWidth > 0, "Viewport width is 0");
        GZ_ASSERT(viewportHeight > 0, "Viewport height is 0");

        // convert to normalized device coordinates (needed by shaders)
        // keep z for visibility test
        lightPos.x = 2.0 * (imagePos.X() / viewportWidth  - 0.5);
        lightPos.y = 2.0 * (1.0 - (imagePos.Y() / viewportHeight) - 0.5);
        // imagePos.Z() is the distance of point from camera optical center
        // if it's > 1.0 than the point is outside of camera view
        // but allow some tol to avoid sharp dropoff of lens flare at
        // edge of image frame. tol = 0.75
        lightPos.z = (imagePos.Z() > 1.75) ? -1 : 1;

        // check occlusion and set scale
        // loop through all env cameras and find the cam that sees the light
        // ray cast using that env camera to see if the distance to closest
        // intersection point is less than light's world pos
        double occlusionScale = 1.0;
        if (lightPos.z >= 0.0)
        {
          // loop through all env cameras
          for (auto cam : ogreEnvCameras)
          {
            // project light world point to camera clip space.
            auto viewProj = cam->getProjectionMatrix() * cam->getViewMatrix();
            auto pos = viewProj *
                Ogre::Vector4(Conversions::Convert(this->lightWorldPos));
            pos.x /= pos.w;
            pos.y /= pos.w;
            // check if light is visible
            if (std::fabs(pos.x) <= 1 && std::fabs(pos.y) <= 1 && pos.z > 0)
            {
              // check occlusion using this env camera
              this->wideAngleDummyCamera->SetWorldPose(ignition::math::Pose3d(
                  Conversions::ConvertIgn(cam->getDerivedPosition()),
                  Conversions::ConvertIgn(cam->getDerivedOrientation())));

              occlusionScale = this->OcclusionScale(
                  this->wideAngleDummyCamera,
                  ignition::math::Vector3d(pos.x, pos.y, pos.z),
                  this->lightWorldPos);
              break;
            }
          }
        }
        _pos = Conversions::ConvertIgn(lightPos);
        _scale = occlusionScale * this->scale;
      }

      /// \brief Check to see if the lens flare is occluded and return a scaling
      /// factor that is proportional to the lens flare's visibility
      /// \param[in] _cam Camera used for checking occlusion
      /// \param[in] _imgPos light pos in clip space
      /// \param[in] _worldPos light pos in 3D world space
      private: double OcclusionScale(const CameraPtr &_cam,
                                     const ignition::math::Vector3d &_imgPos,
                                     const ignition::math::Vector3d &_worldPos)
      {
        double viewportWidth =
            static_cast<double>(_cam->ViewportWidth());
        double viewportHeight =
           static_cast<double>(_cam->ViewportHeight());
        ignition::math::Vector2i screenPos;
        screenPos.X() = ((_imgPos.X() / 2.0) + 0.5) * viewportWidth;
        screenPos.Y() = (1 - ((_imgPos.Y() / 2.0) + 0.5)) * viewportHeight;

        ScenePtr scene = _cam->GetScene();

        // check center point
        // if occluded than set scale to 0
        ignition::math::Vector3d position;
        bool intersect = scene->FirstContact(_cam, screenPos, position);
        if (intersect && (position.Length() < _worldPos.Length()))
          return 0;

        unsigned int rays = 0;
        unsigned int occluded = 0u;
        // work in normalized device coordinates
        // lens flare's halfSize is just an approximated value
        double halfSize = 0.05 * this->scale;
        double steps = 10;
        double stepSize = halfSize * 2 / steps;
        double cx = _imgPos.X();
        double cy = _imgPos.Y();
        double startx = cx - halfSize;
        double starty = cy - halfSize;
        double endx = cx + halfSize;
        double endy = cy + halfSize;
        // do sparse ray cast occlusion check
        for (double i = starty; i < endy; i+=stepSize)
        {
          for (double j = startx; j < endx; j+=stepSize)
          {
            screenPos.X() = ((j / 2.0) + 0.5) * viewportWidth;
            screenPos.Y() = (1 - ((i / 2.0) + 0.5)) * viewportHeight;
            intersect = scene->FirstContact(_cam, screenPos, position);
            if (intersect && (position.Length() < _worldPos.Length()))
              occluded++;
            rays++;
          }
        }
        double s = static_cast<double>(rays - occluded) /
            static_cast<double>(rays);
        return s;
      };

      /// \brief Pointer to light
      private: LightPtr light;

      /// \brief Pointer to camera
      private: CameraPtr camera;

      /// \brief Dummy camera used by wide angle camera for occlusion checking
      private: CameraPtr wideAngleDummyCamera;

      /// \brief Position of light in world frame
      private: ignition::math::Vector3d lightWorldPos;

      /// \brief Scale of lens flare.
      private: double scale = 1.0;
    };

    /// \brief Private data class for LensFlare
    class LensFlarePrivate
    {
      /// \brief Pointer to ogre lens flare compositor instance
      public: Ogre::CompositorInstance *lensFlareInstance = nullptr;

      /// \brief Pointer to ogre lens flare compositor listener
      public: std::shared_ptr<LensFlareCompositorListener>
          lensFlareCompositorListener;

      /// \brief Pointer to camera
      public: CameraPtr camera;

      /// \brief Name of directional light
      public: std::string lightName;

      /// \brief Flag to indicate whether or not to remove lens flare effect.
      public: bool removeLensFlare = false;

      /// \brief Mutex to protect handling of light deletion
      public: std::mutex mutex;

      /// \brief Communication Node
      public: transport::NodePtr node;

      /// \brief Subscribe to the request topic
      public: transport::SubscriberPtr requestSub;

      /// \brief Connection for the pre render event.
      public: event::ConnectionPtr preRenderConnection;

      /// \brief Scale of lens flare.
      public: double lensFlareScale = 1.0;
    };
  }
}

using namespace gazebo;
using namespace rendering;

//////////////////////////////////////////////////
LensFlare::LensFlare()
  : dataPtr(new LensFlarePrivate)
{
}

//////////////////////////////////////////////////
LensFlare::~LensFlare()
{
}

//////////////////////////////////////////////////
void LensFlare::SetCamera(CameraPtr _camera)
{
  if (!_camera)
  {
    gzerr << "Unable to apply lens flare, camera is NULL" << std::endl;
    return;
  }

  this->dataPtr->camera = _camera;
  this->dataPtr->preRenderConnection = event::Events::ConnectPreRender(
      std::bind(&LensFlare::Update, this));
}

//////////////////////////////////////////////////
void LensFlare::SetScale(const double _scale)
{
  this->dataPtr->lensFlareScale = std::max(0.0, _scale);
  if (this->dataPtr->lensFlareCompositorListener)
  {
    this->dataPtr->lensFlareCompositorListener->SetScale(
        this->dataPtr->lensFlareScale);
  }
}

//////////////////////////////////////////////////
void LensFlare::Update()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // remove lens flare if we got a delete msg
  if (this->dataPtr->removeLensFlare)
  {
    this->dataPtr->requestSub.reset();
    this->dataPtr->lensFlareInstance->setEnabled(false);
    this->dataPtr->removeLensFlare = false;
    this->dataPtr->lightName = "";
    return;
  }


  // Get the first directional light
  LightPtr directionalLight;
  for (unsigned int i = 0; i < this->dataPtr->camera->GetScene()->LightCount();
      ++i)
  {
    LightPtr light = this->dataPtr->camera->GetScene()->LightByIndex(i);
    if (light->Type() == "directional")
    {
      directionalLight = light;
      break;
    }
  }
  if (!directionalLight)
    return;

  this->dataPtr->lightName = directionalLight->Name();

  if (!this->dataPtr->lensFlareInstance)
  {
    // set up the lens flare instance
    Ogre::MaterialPtr lensFlareMaterial =
        Ogre::MaterialManager::getSingleton().getByName(
            "Gazebo/CameraLensFlare");
    lensFlareMaterial = lensFlareMaterial->clone(
            "Gazebo/" + this->dataPtr->camera->Name() + "_CameraLensFlare");

    this->dataPtr->lensFlareCompositorListener.reset(new
          LensFlareCompositorListener(this->dataPtr->camera, directionalLight));
    this->dataPtr->lensFlareCompositorListener->SetScale(
        this->dataPtr->lensFlareScale);

    this->dataPtr->lensFlareInstance =
        Ogre::CompositorManager::getSingleton().addCompositor(
        this->dataPtr->camera->OgreViewport(), "CameraLensFlare/Default");
    this->dataPtr->lensFlareInstance->getTechnique()->getOutputTargetPass()->
        getPass(0)->setMaterial(lensFlareMaterial);

    this->dataPtr->lensFlareInstance->setEnabled(true);
    this->dataPtr->lensFlareInstance->addListener(
        this->dataPtr->lensFlareCompositorListener.get());
  }
  else
  {
    this->dataPtr->lensFlareCompositorListener->SetLight(directionalLight);
    this->dataPtr->lensFlareInstance->setEnabled(true);
  }

  // disconnect
  this->dataPtr->preRenderConnection.reset();

  if (!this->dataPtr->node)
  {
    this->dataPtr->node = transport::NodePtr(new transport::Node());
    this->dataPtr->node->Init();
  }

  // listen for delete events to remove lens flare if light gets deleted.
  this->dataPtr->requestSub = this->dataPtr->node->Subscribe("~/request",
      &LensFlare::OnRequest, this);
}

//////////////////////////////////////////////////
void LensFlare::OnRequest(ConstRequestPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if (_msg->request() == "entity_delete" &&
      _msg->data() == this->dataPtr->lightName)
  {
    this->dataPtr->removeLensFlare = true;
    this->dataPtr->preRenderConnection = event::Events::ConnectPreRender(
      std::bind(&LensFlare::Update, this));
  }
}
