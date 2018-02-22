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
    class OcclusionQuery
      : public Ogre::RenderTargetListener,
        public Ogre::RenderObjectListener
    {
      public: OcclusionQuery(CameraPtr _cam, VisualPtr _target)
      {
        this->camera = _cam;
        this->target = _target;
        static unsigned int occlusionQueryId = 0;
        std::string queryVisualName =
            "occlusion_query_" + std::to_string(occlusionQueryId++);
        this->queryVisual.reset(new Visual(queryVisualName, this->target));
        this->queryVisual->Load();
        // this->queryVisual->SetWorldPose(this->target->GetWorldPose());
        this->queryVisual->SetWorldPosition(ignition::math::Vector3d(3.0, 0, 0.5));

        // Create occlusion queries
        try
        {
          Ogre::RenderSystem *renderSystem =
              Ogre::Root::getSingleton().getRenderSystem();
          this->queryArea =
              renderSystem->createHardwareOcclusionQuery();
          this->queryVisible =
              renderSystem->createHardwareOcclusionQuery();
        }
        catch (Ogre::Exception e)
        {
          gzerr << "Failed to create occlusion query: "
                << e.what() << std::endl;
          return;
        }

        // Create the materials to be used by the objects used fo the occlusion query
        Ogre::MaterialPtr matBase =
            Ogre::MaterialManager::getSingleton().getByName(
            "BaseWhiteNoLighting");

        // Not occluded by objects
        std::string queryAreaMatName = "query_area_mat_" +
            std::to_string(occlusionQueryId);
        Ogre::MaterialPtr matQueryArea = matBase->clone(queryAreaMatName);
        matQueryArea->setDepthWriteEnabled(false);
        matQueryArea->setColourWriteEnabled(false);
        matQueryArea->setDepthCheckEnabled(false);

        // Occluded by objects
        std::string queryVisibleMatName = "query_visible_mat_" +
            std::to_string(occlusionQueryId);
        Ogre::MaterialPtr matQueryVisible = matBase->clone(queryVisibleMatName);
        matQueryVisible->setDepthWriteEnabled(false);
        matQueryVisible->setColourWriteEnabled(false);
        matQueryVisible->setDepthCheckEnabled(true);

        Ogre::SceneManager *sceneMgr =
            this->camera->GetScene()->OgreSceneManager();
        // Attach a billboard which will be used to get a relative area
        // occupied by the target node, e.g. light
        this->queryAreaBB = sceneMgr->createBillboardSet(1);
        this->queryAreaBB->setDefaultDimensions(this->size, this->size);
        this->queryAreaBB->createBillboard(Ogre::Vector3::ZERO);
        this->queryAreaBB->setMaterialName(queryAreaMatName);
        // queryAreaBB->setRenderQueueGroup(cPriorityQuery);
        queryAreaBB->setRenderQueueGroup(51);
        queryVisual->AttachObject(this->queryAreaBB);

        // Attach a billboard which will be used to get the visible area
        // occupied by the target node, e.g. light
        this->queryVisibleBB = sceneMgr->createBillboardSet(1);
        this->queryVisibleBB->setDefaultDimensions(this->size, this->size);
        this->queryVisibleBB->createBillboard(Ogre::Vector3::ZERO);
        this->queryVisibleBB->setMaterialName(queryVisibleMatName);
//        this->queryVisibleBB->setRenderQueueGroup(cPriorityQuery);
        queryVisibleBB->setRenderQueueGroup(51);
        this->queryVisual->AttachObject(this->queryVisibleBB);

        Ogre::RenderTarget *ogreRT = this->camera->OgreViewport()->getTarget();
        ogreRT->addListener(this);
        sceneMgr->addRenderObjectListener(this);
        this->doOcclusionQuery = true;
      }

      /// \brief Destructor
      public: ~OcclusionQuery()
      {
        Ogre::RenderSystem *renderSystem =
            Ogre::Root::getSingleton().getRenderSystem();

        if (this->queryArea)
          renderSystem->destroyHardwareOcclusionQuery(this->queryArea);
        if (this->queryVisible)
          renderSystem->destroyHardwareOcclusionQuery(this->queryVisible);
      }

      public: void SetSize(const double _size)
      {
        if (_size <= 0)
        {
          gzerr << "Size must be greater than 0" << std::endl;
          return;
        }
        this->size = _size;
      }

      public: virtual void preRenderTargetUpdate(
                  const Ogre::RenderTargetEvent &/*_evt*/)
      {
        std::cerr << "pre =========" << std::endl;
        this->renderTargetActive = true;
      }

      public: virtual void postRenderTargetUpdate(
                  const Ogre::RenderTargetEvent &/*_evt*/)
      {
        std::cerr << "post ============" << std::endl;
        this->renderTargetActive = false;
      }

      // Event raised when render single object started.
      public: virtual void notifyRenderSingleObject(Ogre::Renderable* _rend,
          const Ogre::Pass * /*_pass*/,
          const Ogre::AutoParamDataSource * /*_source*/,
          const Ogre::LightList * /*_pLightList*/,
          bool /*_suppressRenderStateChanges*/)
      {
        if (!this->renderTargetActive)
          return;

        std::cerr << _rend->getMaterial()->getName() << std::endl;

        //
        // The following code activates and deactivates the occlusion queries
        // so that the queries only include the rendering of their intended targets
        //

        // Close the last occlusion query
        // Each occlusion query should only last a single rendering
        if (this->activeQuery)
        {
          std::cerr << "end occlusion query " << std::endl;
          this->activeQuery->endOcclusionQuery();
          this->activeQuery = nullptr;
        }

        // Open a new occlusion query
        if (this->doOcclusionQuery)
        {
          // Check if a the object being rendered needs
          // to be occlusion queried, and by which query instance.
          if (_rend == this->queryAreaBB)
          {
            this->activeQuery = this->queryArea;
          }
          else if (_rend == this->queryVisibleBB)
          {
            this->activeQuery = this->queryVisible;
          }
          if (this->activeQuery)
          {
            std::cerr << "begin occlusion query " << std::endl;
            this->activeQuery->beginOcclusionQuery();
          }
        }
      }

      public: double OcclusionRatio()
      {
        double ratio = -1;

        this->doOcclusionQuery = false;
        if ((!this->queryArea->isStillOutstanding()) &&
            (!this->queryVisible->isStillOutstanding()))
        {
          unsigned int areaCount = 0;
          unsigned int visibleCount = 0;
          this->queryArea->pullOcclusionQuery(&areaCount);
          this->queryVisible->pullOcclusionQuery(&visibleCount);
          ratio = static_cast<float>(visibleCount) /
              static_cast<float>(areaCount);
          std::cerr << visibleCount << " / " << areaCount << std::endl;

          this->doOcclusionQuery = true;
        }
        return ratio;
      }

      private: Ogre::HardwareOcclusionQuery *queryArea = nullptr;
      private: Ogre::HardwareOcclusionQuery *queryVisible = nullptr;
      private: Ogre::HardwareOcclusionQuery *activeQuery = nullptr;

      private: Ogre::BillboardSet *queryAreaBB = nullptr;
      private: Ogre::BillboardSet *queryVisibleBB = nullptr;

      private: bool doOcclusionQuery = false;
      private: bool renderTargetActive = false;
      private: CameraPtr camera;
      private: VisualPtr target;
      private: VisualPtr queryVisual;
      private: double size = 10;
    };


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

      /// \brief Destructor
      public: ~LensFlareCompositorListener()
      {
        delete this->lensFlareQuery;
      }

      /// \brief Set directional light that generates lens flare
      /// \param[in] _light Pointer to directional light
      public: void SetLight(LightPtr _light)
      {
        this->dir = ignition::math::Quaterniond(_light->Rotation()) *
            _light->Direction();
        // set light world pos to be far away
        this->lightWorldPos = -this->dir * 1000000.0;

        // create dummy light visual for occlusion query
        static unsigned int dummyLightVisId = 0;
        VisualPtr lightVis(
            new Visual(_light->Name() + std::to_string(dummyLightVisId++),
            this->camera->GetScene()->WorldVisual()));
        lightVis->Load();
        lightVis->SetWorldPosition(this->lightWorldPos);

        this->lensFlareQuery = new OcclusionQuery(this->camera, lightVis);
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
          auto imagePos = wideAngleCam->Project3d(this->lightWorldPos);

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
            occlusionScale = this->OcclusionScale(this->camera,
                Conversions::ConvertIgn(lightPos), this->lightWorldPos);

            double ratio = this->lensFlareQuery->OcclusionRatio();
            std::cerr << "ratio " << ratio << std::endl;
          }
          params->setNamedConstant("scale",
              static_cast<Ogre::Real>(occlusionScale));
        }
        params->setNamedConstant("lightPos", lightPos);
      }

     /// \brief Check to see if the lensflare is occluded and return a scaling
     /// factor for the lens flare that is proportional to its visibility
     private: double OcclusionScale(CameraPtr _cam,
                                    const ignition::math::Vector3d &_imgPos,
                                    const ignition::math::Vector3d &_lightPos)
     {
       double viewportWidth =
           static_cast<double>(_cam->ViewportWidth());
       double viewportHeight =
          static_cast<double>(_cam->ViewportHeight());
       ignition::math::Vector2i screenPos;
       screenPos.X() = ((_imgPos.X() / 2.0) + 0.5) * viewportWidth;
       screenPos.Y() = (1 - ((_imgPos.Y() / 2.0) + 0.5)) * viewportHeight;
       ScenePtr scene = _cam->GetScene();

       unsigned int rays = 0;
       unsigned int occluded = 0u;
       // work in normalized device coordinates
       // lens flare's halfSize is just an approximated value
       double halfSize = 0.065;
       double steps = 5;
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
           ignition::math::Vector3d position;
           bool intersect = scene->FirstContact(_cam, screenPos, position);
           if (intersect && (position.Length() < _lightPos.Length()))
             occluded++;
           rays++;
         }
       }
       double s = static_cast<double>(rays - occluded) /
           static_cast<double>(rays);
       std::cerr << "scale s: " << s << " [" << occluded << "/" << rays << "]" <<  std::endl;
       return s;
     };

      /// \brief Pointer to camera
      private: CameraPtr camera;

      /// \brief Light dir in world frame
      private: ignition::math::Vector3d dir;

      /// \brief Light pos in world frame
      private: ignition::math::Vector3d lightWorldPos;

      private: OcclusionQuery *lensFlareQuery = nullptr;
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
    LightPtr light = this->dataPtr->camera->GetScene()->GetLight(i);
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
