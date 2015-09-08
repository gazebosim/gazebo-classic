/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#if defined(HAVE_OPENGL)
#include <GL/gl.h>
#include <GL/glext.h>
#endif


#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/CameraLensPrivate.hh"
#include "gazebo/rendering/WideAngleCameraPrivate.hh"
#include "gazebo/rendering/skyx/include/SkyX.h"

#include "gazebo/rendering/RTShaderSystem.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/WideAngleCamera.hh"


using namespace gazebo;
using namespace rendering;

//////////////////////////////////////////////////
CameraLens::CameraLens()
{
  this->dataPtr = new CameraLensPrivate;
}

//////////////////////////////////////////////////
CameraLens::~CameraLens()
{
  delete this->dataPtr;
}

//////////////////////////////////////////////////
void CameraLens::Init(double _c1, double _c2, const std::string &_fun,
                      double _f, double _c3)
{
  this->dataPtr->c1 = _c1;
  this->dataPtr->c2 = _c2;
  this->dataPtr->c3 = _c3;
  this->dataPtr->f = _f;

  try
  {
    this->dataPtr->fun = CameraLensPrivate::MapFunctionEnum(_fun);
  }
  catch(...)
  {
    std::stringstream sstr;
    sstr << "Failed to create custom mapping with function [" << _fun << "]";

    gzthrow(sstr.str());
  }
}

//////////////////////////////////////////////////
void CameraLens::Init(const std::string &_name)
{
  this->SetType(_name);
}

//////////////////////////////////////////////////
void CameraLens::Load(sdf::ElementPtr _sdf)
{
  this->dataPtr->sdf = _sdf;

  this->Load();
}

//////////////////////////////////////////////////
void CameraLens::Load()
{
  if (!this->dataPtr->sdf->HasElement("type"))
    gzthrow("You should specify lens type using <type> element");

  if (this->IsCustom())
  {
    if (this->dataPtr->sdf->HasElement("custom_function"))
    {
      sdf::ElementPtr cf = this->dataPtr->sdf->GetElement("custom_function");

      this->Init(
        cf->Get<double>("c1"),
        cf->Get<double>("c2"),
        cf->Get<std::string>("fun"),
        cf->Get<double>("f"),
        cf->Get<double>("c3"));
    }
    else
      gzthrow("You need a <custom_function> element to use this lens type");
  }
  else
    this->Init(this->Type());

  this->SetCutOffAngle(this->dataPtr->sdf->Get<double>("cutoff_angle"));
}

//////////////////////////////////////////////////
std::string CameraLens::Type() const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->dataMutex);

  return this->dataPtr->sdf->Get<std::string>("type");
}

//////////////////////////////////////////////////
bool CameraLens::IsCustom() const
{
  return this->Type() == "custom";
}

//////////////////////////////////////////////////
double CameraLens::C1() const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->dataMutex);

  return this->dataPtr->c1;
}

//////////////////////////////////////////////////
double CameraLens::C2() const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->dataMutex);

  return this->dataPtr->c2;
}

//////////////////////////////////////////////////
double CameraLens::C3() const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->dataMutex);

  return this->dataPtr->c3;
}

//////////////////////////////////////////////////
double CameraLens::F() const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->dataMutex);

  return this->dataPtr->f;
}

//////////////////////////////////////////////////
std::string CameraLens::Fun() const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->dataMutex);

  return this->dataPtr->fun.AsString();
}

//////////////////////////////////////////////////
double CameraLens::CutOffAngle() const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->dataMutex);

  return this->dataPtr->cutOffAngle;
}

//////////////////////////////////////////////////
bool CameraLens::ScaleToHFOV() const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->dataMutex);

  return this->dataPtr->sdf->Get<bool>("scale_to_hfov");
}

//////////////////////////////////////////////////
void CameraLens::SetType(const std::string &_type)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->dataMutex);

  // c1, c2, c3, f, fun
  std::map< std::string, std::tuple<double, double, double,
      double, std::string> > fun_types = {
    {"gnomonical",      std::make_tuple(1.0, 1.0, 0.0, 1.0, "tan")},
    {"stereographic",   std::make_tuple(2.0, 2.0, 0.0, 1.0, "tan")},
    {"equidistant",     std::make_tuple(1.0, 1.0, 0.0, 1.0, "id")},
    {"equisolid_angle", std::make_tuple(2.0, 2.0, 0.0, 1.0, "sin")},
    {"orthographic",    std::make_tuple(1.0, 1.0, 0.0, 1.0, "sin")}};

  fun_types.emplace("custom",
      std::make_tuple(this->C1(), this->C2(), this->C3(), this->F(),
        CameraLensPrivate::MapFunctionEnum(this->Fun()).AsString()));

  decltype(fun_types)::mapped_type params;

  try
  {
    params = fun_types.at(_type);
  }
  catch(...)
  {
    std::stringstream sstr;
    sstr << "Unknown mapping function [" << _type << "]";

    gzthrow(sstr.str());
  }

  this->dataPtr->sdf->GetElement("type")->Set(_type);

  if (_type == "custom")
  {
    this->SetC1(std::get<0>(params));
    this->SetC2(std::get<1>(params));
    this->SetC3(std::get<2>(params));
    this->SetF(std::get<3>(params));
    this->SetFun(std::get<4>(params));
  }
  else
  {
    // Do not write values to SDF
    this->dataPtr->c1 = std::get<0>(params);
    this->dataPtr->c2 = std::get<1>(params);
    this->dataPtr->c3 = std::get<2>(params);
    this->dataPtr->f = std::get<3>(params);
    this->dataPtr->fun =
        CameraLensPrivate::MapFunctionEnum(std::get<4>(params));
  }
}

//////////////////////////////////////////////////
void CameraLens::SetC1(double _c)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->dataMutex);

  this->dataPtr->c1 = _c;

  if (!this->IsCustom())
    this->ConvertToCustom();

  this->dataPtr->sdf->GetElement("custom_function")->GetElement("c1")->Set(_c);
}

//////////////////////////////////////////////////
void CameraLens::SetC2(double _c)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->dataMutex);

  this->dataPtr->c2 = _c;

  if (!this->IsCustom())
    this->ConvertToCustom();

  this->dataPtr->sdf->GetElement("custom_function")->GetElement("c2")->Set(_c);
}

//////////////////////////////////////////////////
void CameraLens::SetC3(double _c)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->dataMutex);

  this->dataPtr->c3 = _c;

  if (!this->IsCustom())
    this->ConvertToCustom();

  this->dataPtr->sdf->GetElement("custom_function")->GetElement("c3")->Set(_c);
}

//////////////////////////////////////////////////
void CameraLens::SetF(double _f)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->dataMutex);

  this->dataPtr->f = _f;

  if (!this->IsCustom())
    this->ConvertToCustom();

  this->dataPtr->sdf->GetElement("custom_function")->GetElement("f")->Set(_f);
}

void CameraLens::SetFun(const std::string &_fun)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->dataMutex);

  if (!this->IsCustom())
    this->ConvertToCustom();

  try
  {
    this->dataPtr->fun = CameraLensPrivate::MapFunctionEnum(_fun);
  }
  catch(...)
  {
    std::stringstream sstr;
    sstr << "Failed to create custom mapping with function [" << _fun << "]";

    gzthrow(sstr.str());
  }

  auto customFunction = this->dataPtr->sdf->GetElement("custom_function");
  customFunction->GetElement("fun")->Set(_fun);
}

//////////////////////////////////////////////////
void CameraLens::SetCutOffAngle(double _angle)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->dataMutex);

  this->dataPtr->cutOffAngle = _angle;

  this->dataPtr->sdf->GetElement("cutoff_angle")->Set(_angle);
}

//////////////////////////////////////////////////
void CameraLens::SetScaleToHFOV(bool _scale)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->dataMutex);

  this->dataPtr->sdf->GetElement("scale_to_hfov")->Set(_scale);
}

//////////////////////////////////////////////////
void CameraLens::SetUniformVariables(Ogre::Pass *_pass,
    float _ratio, float _hfov)
{
  Ogre::GpuProgramParametersSharedPtr uniforms =
    _pass->getFragmentProgramParameters();

  uniforms->setNamedConstant("c1", static_cast<Ogre::Real>(this->dataPtr->c1));
  uniforms->setNamedConstant("c2", static_cast<Ogre::Real>(this->dataPtr->c2));
  uniforms->setNamedConstant("c3", static_cast<Ogre::Real>(this->dataPtr->c3));

  if (this->ScaleToHFOV())
  {
    float param = (_hfov/2)/this->dataPtr->c2+this->dataPtr->c3;
    float fun_res = this->dataPtr->fun.Apply(static_cast<float>(param));

    float new_f = 1.0f/(this->dataPtr->c1*fun_res);

    uniforms->setNamedConstant("f", static_cast<Ogre::Real>(new_f));
  }
  else
    uniforms->setNamedConstant("f", static_cast<Ogre::Real>(this->dataPtr->f));

  auto vec_fun = this->dataPtr->fun.AsVector3d();

  uniforms->setNamedConstant("fun", Ogre::Vector3(
      vec_fun.X(), vec_fun.Y(), vec_fun.Z()));

  uniforms->setNamedConstant("cutOffAngle",
    static_cast<Ogre::Real>(this->dataPtr->cutOffAngle));

  Ogre::GpuProgramParametersSharedPtr uniforms_vs =
    _pass->getVertexProgramParameters();

  uniforms_vs->setNamedConstant("ratio", static_cast<Ogre::Real>(_ratio));
}

//////////////////////////////////////////////////
void CameraLens::ConvertToCustom()
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->dataMutex);

  this->SetType("custom");

  sdf::ElementPtr cf = this->dataPtr->sdf->AddElement("custom_function");

  cf->AddElement("c1")->Set(this->dataPtr->c1);
  cf->AddElement("c2")->Set(this->dataPtr->c2);
  cf->AddElement("c3")->Set(this->dataPtr->c3);
  cf->AddElement("f")->Set(this->dataPtr->f);

  cf->AddElement("fun")->Set(this->dataPtr->fun.AsString());
}

//////////////////////////////////////////////////
WideAngleCamera::WideAngleCamera(const std::string &_namePrefix,
    ScenePtr _scene, bool _autoRender, int _textureSize):
  Camera(_namePrefix, _scene, _autoRender)
{
  this->dataPtr = new WideAngleCameraPrivate;
  this->lens = new CameraLens();

  envCubeMapTexture = NULL;
  this->dataPtr->envTextureSize = _textureSize;

  for (int i = 0; i < 6; ++i)
  {
    envCameras[i] = NULL;
    envRenderTargets[i] = NULL;
  }
}

//////////////////////////////////////////////////
WideAngleCamera::~WideAngleCamera()
{
  delete this->lens;
  delete this->dataPtr;
}

//////////////////////////////////////////////////
void WideAngleCamera::Init()
{
  Camera::Init();

  for (int i = 0; i < 6; ++i)
    this->sceneNode->attachObject(this->envCameras[i]);

  // set environment cameras orientation
  this->envCameras[0]->yaw(Ogre::Degree(-90));
  this->envCameras[1]->yaw(Ogre::Degree(90));
  this->envCameras[2]->pitch(Ogre::Degree(90));
  this->envCameras[3]->pitch(Ogre::Degree(-90));
  this->envCameras[5]->yaw(Ogre::Degree(180));

  this->CreateEnvRenderTexture(this->scopedUniqueName + "_envRttTex");
}

//////////////////////////////////////////////////
void WideAngleCamera::Load()
{
  Camera::Load();

  this->CreateEnvCameras();

  if (this->sdf->HasElement("lens"))
  {
    sdf::ElementPtr sdf_lens = this->sdf->GetElement("lens");

    this->lens->Load(sdf_lens);

    if (sdf_lens->HasElement("env_texture_size"))
      this->dataPtr->envTextureSize = sdf_lens->Get<int>("env_texture_size");
  }
  else
    this->lens->Load();
}

//////////////////////////////////////////////////
void WideAngleCamera::Fini()
{
  for (int i = 0; i < 6; ++i)
  {
    RTShaderSystem::DetachViewport(this->envViewports[i], this->GetScene());

    this->envRenderTargets[i]->removeAllViewports();
    this->envRenderTargets[i] = NULL;

    this->GetScene()->GetManager()->destroyCamera(envCameras[i]->getName());
    envCameras[i] = NULL;
  }

  if (this->envCubeMapTexture)
    Ogre::TextureManager::getSingleton().remove(
      this->envCubeMapTexture->getName());

  this->envCubeMapTexture = NULL;

  Camera::Fini();
}

//////////////////////////////////////////////////
int WideAngleCamera::EnvTextureSize() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->dataMutex);

  return this->dataPtr->envTextureSize;
}

//////////////////////////////////////////////////
CameraLens *WideAngleCamera::Lens()
{
  return this->lens;
}

//////////////////////////////////////////////////
void WideAngleCamera::SetRenderTarget(Ogre::RenderTarget *_target)
{
  Camera::SetRenderTarget(_target);

  if (this->renderTarget)
  {
    this->cubeMapCompInstance =
      Ogre::CompositorManager::getSingleton().addCompositor(this->viewport,
          "WideCameraLensMap/ParametrisedMap");

    if (this->envCubeMapTexture)
    {
      this->compMat =
          Ogre::MaterialManager::getSingleton().getByName("Gazebo/WideLensMap");

      auto pass = this->compMat->getTechnique(0)->getPass(0);

      if (!pass->getNumTextureUnitStates())
        pass->createTextureUnitState();

      this->cubeMapCompInstance->addListener(this);
    }
    else
      gzerr << "Compositor texture MISSING";


    this->cubeMapCompInstance->setEnabled(true);
  }
}

//////////////////////////////////////////////////
void WideAngleCamera::SetEnvTextureSize(int _size)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->dataMutex);

  if (this->sdf->HasElement("env_texture_size"))
    this->sdf->AddElement("env_texture_size")->Set(_size);

  this->sdf->GetElement("env_texture_size")->Set(_size);
}

//////////////////////////////////////////////////
void WideAngleCamera::CreateEnvCameras()
{
  for (int i = 0; i < 6; ++i)
  {
    std::stringstream name_str;

    name_str << this->scopedUniqueName << "_env_" << i;

    this->envCameras[i] =
        this->GetScene()->GetManager()->createCamera(name_str.str());

    this->envCameras[i]->setFixedYawAxis(false);
    this->envCameras[i]->setFOVy(Ogre::Degree(90));
    this->envCameras[i]->setAspectRatio(1);

    this->envCameras[i]->yaw(Ogre::Degree(-90.0));
    this->envCameras[i]->roll(Ogre::Degree(-90.0));

    this->envCameras[i]->setNearClipDistance(0.01);
    this->envCameras[i]->setFarClipDistance(1000);
  }
}

//////////////////////////////////////////////////
void WideAngleCamera::SetClipDist()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->dataMutex);

  sdf::ElementPtr clipElem = this->sdf->GetElement("clip");
  if (!clipElem)
    gzthrow("Camera has no <clip> element.");

  for (int i = 0; i < 6; ++i)
  {
    if (this->envCameras[i])
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

//////////////////////////////////////////////////
void WideAngleCamera::CreateEnvRenderTexture(const std::string &_textureName)
{
  int fsaa = 4;

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR < 8
  fsaa = 0;
#endif

  this->envCubeMapTexture = Ogre::TextureManager::getSingleton().createManual(
      this->scopedUniqueName+"::"+_textureName,
      "General",
      Ogre::TEX_TYPE_CUBE_MAP,
      this->dataPtr->envTextureSize,
      this->dataPtr->envTextureSize,
      0,
      Ogre::PF_A8R8G8B8,
      Ogre::TU_RENDERTARGET,
      0,
      false,
      fsaa).getPointer();

  for (int i = 0; i < 6; ++i)
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
    vp->setVisibilityMask(GZ_VISIBILITY_ALL &
        ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE));

    this->envViewports[i] = vp;

    if (this->GetScene()->GetSkyX())
      rtt->addListener(this->GetScene()->GetSkyX());

    this->envRenderTargets[i] = rtt;
  }
}

//////////////////////////////////////////////////
void WideAngleCamera::RenderImpl()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->renderMutex);

  for (int i = 0; i < 6; ++i)
    this->envRenderTargets[i]->update();

  compMat->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTextureName(
    this->envCubeMapTexture->getName());

  this->renderTarget->update();
}

//////////////////////////////////////////////////
void WideAngleCamera::notifyMaterialRender(Ogre::uint32 /*_pass_id*/,
    Ogre::MaterialPtr &_material)
{
  if (_material.isNull())
    return;

  Ogre::Technique *pTechnique = _material->getBestTechnique();
  if (!pTechnique)
    return;

  Ogre::Pass *pPass = pTechnique->getPass(0);
  if (!pPass || !pPass->hasFragmentProgram())
    return;

  if (!this->Lens())
  {
    gzerr << "No lens\n";
    return;
  }

  this->Lens()->SetUniformVariables(pPass,
    this->GetAspectRatio(),
    this->GetHFOV().Radian());

#if defined(HAVE_OPENGL)
  // XXX: OGRE doesn't allow to enable cubemap filtering extention thru it's API
  // suppose that this function was invoked in a thread that has OpenGL context
  glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);
#endif
}
