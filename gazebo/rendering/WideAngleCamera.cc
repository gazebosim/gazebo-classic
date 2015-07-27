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
    sdf::ElementPtr sdf_projection = this->sdf->GetElement("projection");

    this->projection.Load(sdf_projection);

    if(sdf_projection->HasElement("cube_tex_resolution"))
      this->envTextureSize = sdf_projection->Get<int>("cube_tex_resolution");
  }
  else
    this->projection.Load();
}

void WideAngleCamera::Fini()
{
  for(int i=0;i<6;i++)
  {
    RTShaderSystem::DetachViewport(this->envViewports[i], this->GetScene());

    this->envRenderTargets[i]->removeAllViewports();
    this->envRenderTargets[i] = NULL;

    this->GetScene()->GetManager()->destroyCamera(envCameras[i]->getName());
    envCameras[i] = NULL;
  }

  if(this->envCubeMapTexture)
    Ogre::TextureManager::getSingleton().remove(this->envCubeMapTexture->getName());
  this->envCubeMapTexture = NULL;

  Camera::Fini();
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

      this->projection.SetCompositorMaterial(this->compMat);

      gzdbg << "Compositor cubemap texture present " << envCubeMapTexture->getName() << "\n";
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

    this->envViewports[i] = vp;

    //FIXME: problem with Skyx include
    // if (this->GetScene()->GetSkyX())
    //   rtt->addListener(this->GetScene()->GetSkyX());

    this->envRenderTargets[i] = rtt;
  }
}

void WideAngleCamera::SetEnvTextureSize(int size)
{
  if(this->sdf->HasElement("cube_tex_resolution"))
    this->sdf->AddElement("cube_tex_resolution")->Set(size);

  this->sdf->GetElement("cube_tex_resolution")->Set(size);
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

  this->projection.SetMaterialVariables();

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

void CameraProjection::Init(float c1,float c2,std::string fun,float f,float c3)
{
  this->c1 = c1;
  this->c2 = c2;
  this->c3 = c3;
  this->f  = f;

  if(fun == "tan")
    this->fun = TAN;
  else if(fun == "sin")
    this->fun = SIN;
  else if(fun == "id")
    this->fun = ID;
  else if(fun == "cos")
  {
    gzthrow("Cosine is not supported for custom mapping functions");
  }
  else if(fun == "cot")
  {
    gzthrow("Cotangent is not supported for custom mapping functions");
  }
  else
  {
    std::stringstream sstr;
    sstr << "Failed to create custom mapping with function [" << fun << "]";

    gzthrow(sstr.str());
  }
}

void CameraProjection::Init(std::string name)
{
  std::map<std::string,std::tuple<float,float,float,float,std::string> > fun_types;

  // c1,c2,c3,f,fun
  fun_types.emplace("gnomonical",   std::make_tuple(1.0f,1.0f,0.0f,1.0f,"tan"));
  fun_types.emplace("stereographic",  std::make_tuple(2.0f,0.5f,0.0f,5.0f,"tan"));
  fun_types.emplace("equidistant",  std::make_tuple(1.0f,1.0f,0.0f,1.0f,"id"));
  fun_types.emplace("equisolid_angle",std::make_tuple(2.0f,0.5f,0.0f,1.0f,"sin"));
  fun_types.emplace("orthographic", std::make_tuple(1.0f,1.0f,0.0f,1.0f,"sin"));

  std::tuple<float,float,float,float,std::string> params;
  try
  {
    params = fun_types.at(name);
  }
  catch(...)
  {
    std::stringstream sstr;
    sstr << "Unknown mapping function [" << name << "]";

    gzthrow(sstr.str());
  }

  this->Init(
    std::get<0>(params),
    std::get<1>(params),
    std::get<4>(params),
    std::get<3>(params),
    std::get<2>(params));
}

void CameraProjection::Load(sdf::ElementPtr sdf)
{
  this->sdf = sdf;

  Load();
}

void CameraProjection::Load()
{
  if(!this->sdf->HasElement("type"))
    gzthrow("You should specify projection type using <type> element");

  if(this->IsCustom())
  {
    if(this->sdf->HasElement("custom_function"))
    {
      sdf::ElementPtr cf = this->sdf->GetElement("custom_function");

      this->Init(
        cf->Get<double>("c1"),
        cf->Get<double>("c1"),
        cf->Get<std::string>("fun"),
        cf->Get<double>("f"));
    }
    else
      gzthrow("You need a <custom_function> element to use this projection type");
  }
  else
    this->Init(this->GetType());
}

float CameraProjection::GetC1()
{
  return this->c1;
}

float CameraProjection::GetC2()
{
  return this->c2;
}

float CameraProjection::GetC3()
{
  return this->c3;
}

float CameraProjection::GetF()
{
  return this->f;
}

std::string CameraProjection::GetFun()
{
  return this->sdf->GetElement("custom_function")->Get<std::string>("fun");
}

void CameraProjection::SetC1(float c)
{
  this->c1 = c;

  if(!this->IsCustom())
    this->ConvertToCustom();

  this->sdf->GetElement("custom_function")->GetElement("c1")->Set((double)c);
}

void CameraProjection::SetC2(float c)
{
  this->c2 = c;

  if(!this->IsCustom())
    this->ConvertToCustom();

  this->sdf->GetElement("custom_function")->GetElement("c2")->Set((double)c);
}

void CameraProjection::SetC3(float c)
{
  this->c3 = c;

  if(!this->IsCustom())
    this->ConvertToCustom();

  // this->sdf->GetElement("custom_function")->GetElement("c3")->Set((double)c);
}

void CameraProjection::SetF(float f)
{
  this->f = f;

  if(!this->IsCustom())
    this->ConvertToCustom();

  this->sdf->GetElement("custom_function")->GetElement("f")->Set((double)f);
}

void CameraProjection::SetFun(std::string fun)
{
  if(!this->IsCustom())
    this->ConvertToCustom();

  this->sdf->GetElement("custom_function")->GetElement("fun")->Set(fun);

  this->Load();
}

void CameraProjection::ConvertToCustom()
{
  sdf::ElementPtr cf = this->sdf->AddElement("custom_function");

  cf->AddElement("c1")->Set((double)this->c1);
  cf->AddElement("c2")->Set((double)this->c2);
  cf->AddElement("f")->Set((double)this->f);
  cf->AddElement("fun")->Set(std::string("tan"));  //FIXME: choose appropriate string

  this->SetType("custom");
}

std::string CameraProjection::GetType()
{
  return this->sdf->Get<std::string>("type");
}

void CameraProjection::SetType(std::string type)
{
  this->sdf->GetElement("type")->Set(type);
}

bool CameraProjection::IsCustom()
{
  return GetType() == "custom";
}

bool CameraProjection::IsFullFrame()
{
  return this->sdf->Get<bool>("full_frame");
}

void CameraProjection::SetCompositorMaterial(Ogre::MaterialPtr material)
{
  this->compositorMaterial = material;
}

void CameraProjection::SetMaterialVariables()
{
  Ogre::GpuProgramParametersSharedPtr uniforms =
    this->compositorMaterial->getTechnique(0)->getPass(0)->getFragmentProgramParameters();

  // uniforms->setNamedConstant("HFOV",(Ogre::Real)Camera::GetHFOV().Radian());
  // uniforms->setNamedConstant("projectionType",this->projectionType);

  math::Vector3 fun_m[] = {
    math::Vector3(1,0,0),
    math::Vector3(0,1,0),
    math::Vector3(0,0,1)
  };

  uniforms->setNamedConstant("c1",(Ogre::Real)this->c1);
  uniforms->setNamedConstant("c2",(Ogre::Real)this->c2);
  uniforms->setNamedConstant("c3",(Ogre::Real)this->c3);

  uniforms->setNamedConstant("f",(Ogre::Real)this->f);
  uniforms->setNamedConstant("fun",Ogre::Vector3(
      fun_m[this->fun].x,
      fun_m[this->fun].y,
      fun_m[this->fun].z));
}
