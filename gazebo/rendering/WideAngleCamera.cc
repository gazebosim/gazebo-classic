//
// Created by klokik on 02.07.15.
//

#include "WideAngleCamera.hh"

// #include "gazebo/rendering/skyx/include/SkyX.h"
#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/rendering/RTShaderSystem.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"


using namespace gazebo;
using namespace rendering;

WideAngleCamera::WideAngleCamera(const std::string &_namePrefix, ScenePtr _scene, bool _autoRender, int textureSize):
  Camera(_namePrefix,_scene,_autoRender),
  envTextureSize(textureSize)
{
  envCubeMapTexture = NULL;
  for(int i=0;i<6;i++)
  {
    envCameras[i] = NULL;
    envRenderTargets[i] = NULL;
  }
}

WideAngleCamera::~WideAngleCamera()
{
  //TODO
}

void WideAngleCamera::Init()
{
  Camera::Init();

  this->CreateEnvCameras();
}

void WideAngleCamera::Load()
{
  Camera::Load();

  if(this->sdf->HasElement("projection"))
  {
    sdf::ElementPtr elem = this->sdf->GetElement("projection");

    this->projectionType = elem->Get<int>("type");
  }
  else
  {
    this->projectionType = 0;
  }
}

void WideAngleCamera::SetRenderTarget(Ogre::RenderTarget *_target)
{
  Camera::SetRenderTarget(_target);

  if(this->renderTarget)
  {
    gzdbg << "Add Camera Compositor\n";

    this->wamapInstance =
      Ogre::CompositorManager::getSingleton().addCompositor(this->viewport,
          "WideCameraLensMap/PathThrough");

    if(this->envCubeMapTexture)
    {
      this->compMat = Ogre::MaterialManager::getSingleton().getByName("Gazebo/WideLensMap");

      if(compMat->getTechnique(0)->getPass(0)->getNumTextureUnitStates() == 0)
      {
        compMat->getTechnique(0)->getPass(0)->createTextureUnitState();
      }

      gzdbg << "Compositor cubemap texture bound OK " << envCubeMapTexture->getName() << "\n";
    }
    else
      gzerr << "Compositor texture MISSING";


    this->wamapInstance->setEnabled(true);
  }
}

void WideAngleCamera::CreateEnvRenderTexture(const std::string &_textureName)
{
  int fsaa = 4;

  // if(this->envCubeMapTexture)
  // {
  //   delete this->envCubeMapTexture;   // XXX
  //   this->envCubeMapTexture = NULL;
  // }
  gzdbg << "Creating cubemap texture: " << this->scopedUniqueName << "::" << _textureName << "\n";
  gzdbg << "Camera instance name: " << this->GetName() << "\n";

  this->envCubeMapTexture = Ogre::TextureManager::getSingleton().createManual(
      this->scopedUniqueName+"::"+_textureName,
      "General",
      Ogre::TEX_TYPE_CUBE_MAP,
      this->envTextureSize,
      this->envTextureSize,
      0,
      Ogre::PF_A8R8G8B8,
      Ogre::TU_RENDERTARGET,
      0,
      false,
      fsaa).getPointer();

  for(int i=0;i<6;i++)
  {
    Ogre::RenderTarget *rtt;
    rtt = this->envCubeMapTexture->getBuffer(i)->getRenderTarget();

    Ogre::Viewport *vp = rtt->addViewport(this->envCameras[i]);
    vp->setClearEveryFrame(true);
    vp->setShadowsEnabled(true);
    vp->setOverlaysEnabled(false);

    RTShaderSystem::AttachViewport(vp, this->GetScene());

    vp->setBackgroundColour(
      Conversions::Convert(this->scene->GetBackgroundColor()));
    vp->setVisibilityMask(GZ_VISIBILITY_ALL & ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE));

    //FIXME: problem with Skyx include
    // if (this->GetScene()->GetSkyX())
    //   rtt->addListener(this->GetScene()->GetSkyX());

    this->envRenderTargets[i] = rtt;
  }
}

void WideAngleCamera::CreateEnvCameras()
{
  for(int i=0;i<6;i++)
  {
    std::stringstream name_str;

    name_str << this->scopedUniqueName << "_env_" << i;

    envCameras[i] = this->GetScene()->GetManager()->createCamera(name_str.str());

    gzdbg << "creating camera "<< name_str.str() << "\n";

    envCameras[i]->setFixedYawAxis(false);
    envCameras[i]->setFOVy(Ogre::Degree(90));
    envCameras[i]->setAspectRatio(1);

    envCameras[i]->setNearClipDistance(0.01);
    envCameras[i]->setFarClipDistance(1000);
  }
}

void WideAngleCamera::RenderImpl()
{
  math::Quaternion orient = this->GetWorldRotation();
  math::Vector3 pos = this->GetWorldPosition();

  for(int i=0;i<6;i++)
  {
    this->envCameras[i]->setPosition(this->camera->getRealPosition());
    this->envCameras[i]->setOrientation(this->camera->getRealOrientation());
  }

  this->envCameras[0]->rotate(this->camera->getDerivedUp(),Ogre::Degree(-90));
  this->envCameras[1]->rotate(this->camera->getDerivedUp(),Ogre::Degree(90));
  this->envCameras[2]->pitch(Ogre::Degree(90));
  this->envCameras[3]->pitch(Ogre::Degree(-90));
  this->envCameras[5]->rotate(this->camera->getDerivedUp(),Ogre::Degree(180));

  for(int i=0;i<6;i++)
    this->envRenderTargets[i]->update();

  compMat->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTextureName(
    this->envCubeMapTexture->getName());

  Ogre::GpuProgramParametersSharedPtr uniforms =
    compMat->getTechnique(0)->getPass(0)->getFragmentProgramParameters();

  uniforms->setNamedConstant("HFOV",(Ogre::Real)Camera::GetHFOV().Radian());
  uniforms->setNamedConstant("projectionType",this->projectionType);

  this->renderTarget->update();
}

void WideAngleCamera::SetClipDist()
{
  sdf::ElementPtr clipElem = this->sdf->GetElement("clip");
  if (!clipElem)
    gzthrow("Camera has no <clip> tag.");

  for(int i=0;i<6;i++)
  {
    if(this->envCameras[i])
    {
      this->envCameras[i]->setNearClipDistance(clipElem->Get<double>("near"));
      this->envCameras[i]->setFarClipDistance(clipElem->Get<double>("far"));
      this->envCameras[i]->setRenderingDistance(clipElem->Get<double>("far"));
    }
    else
    {
      gzerr << "Setting clip distances failed -- no camera yet\n";
      break;
    }
  }
}