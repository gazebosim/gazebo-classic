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
    /// used to inject dir light clip space pos and time (for animating flare)
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

      /// \brief Set directional light that generates lens flare
      /// \param[in] _light Pointer to directional light
      public: void SetLight(LightPtr _light)
      {
        this->dir = ignition::math::Quaterniond(_light->Rotation()) *
            _light->Direction();
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

        // set light world pos to be far away
        auto worldPos = -this->dir * 100000.0;

        Ogre::Vector3 lightPos;
        // cast to wide angle camera and use project function
        auto wideAngleCam =
            boost::dynamic_pointer_cast<WideAngleCamera>(this->camera);
        if (wideAngleCam)
        {
          // project camera into screen space
          double viewportWidth =
              static_cast<double>(wideAngleCam->ViewportWidth());
          double viewportHeight =
              static_cast<double>(wideAngleCam->ViewportHeight());
          auto imagePos = wideAngleCam->Project3d(worldPos);

          // convert to normalized device coordinates
          // keep z for visibility test
          lightPos.x = 2.0 * (imagePos.X() / viewportWidth  - 0.5);
          lightPos.y = 2.0 * (1.0 - (imagePos.Y() / viewportHeight) - 0.5);
          // imagePos.Z() is the distance of point from camera optical center
          // if it's > 1.0 than the point is outside of camera view
          // but allow some tol to avoid sharp dropoff of lens flare at
          // edge of image frame. tol = 0.75
          lightPos.z = (imagePos.Z() > 1.75) ? -1 : 1;
        }
        else
        {
          // project 3d world space to clip space
          auto viewProj = this->camera->OgreCamera()->getProjectionMatrix() *
            this->camera->OgreCamera()->getViewMatrix();
          auto pos = viewProj * Ogre::Vector4(Conversions::Convert(worldPos));
          // normalize x and y
          // keep z for visibility test
          lightPos.x = pos.x / pos.w;
          lightPos.y = pos.y / pos.w;
          lightPos.z = pos.z;
        }
        params->setNamedConstant("lightPos", lightPos);
      }

      /// \brief Pointer to camera
      private: CameraPtr camera;

      /// \brief Light dir in world frame
      private: ignition::math::Vector3d dir;
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
