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

#if defined(__APPLE__)
#include <OpenGL/gl.h>
#include <OpenGL/glext.h>
#else
#if defined(_WIN32)
  #include <windows.h>
#endif /* _WIN32 */
#include <GL/gl.h>
#include <GL/glext.h>
#endif /* __APPLE__ */

#endif /* HAVE_OPENGL */

#include <ignition/math/Color.hh>

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/CameraLensPrivate.hh"
#include "gazebo/rendering/WideAngleCameraPrivate.hh"
#include "gazebo/rendering/skyx/include/SkyX.h"

#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/RTShaderSystem.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/WideAngleCamera.hh"

using namespace gazebo;
using namespace rendering;

//////////////////////////////////////////////////
CameraLens::CameraLens()
  : dataPtr(new CameraLensPrivate)
{
}

//////////////////////////////////////////////////
CameraLens::~CameraLens()
{
}

//////////////////////////////////////////////////
void CameraLens::Init(const double _c1, const double _c2,
                      const std::string &_fun, const double _f,
                      const double _c3)
{
  this->dataPtr->c1 = _c1;
  this->dataPtr->c2 = _c2;
  this->dataPtr->c3 = _c3;
  this->dataPtr->f = _f;

  try
  {
    this->dataPtr->fun = CameraLensPrivate::MapFunctionEnum(_fun);
  }
  catch(const std::exception &ex)
  {
    gzerr << "`fun` value [" << _fun << "] is not known, "
          << "[tan] will be used instead" << std::endl;

    this->dataPtr->fun = CameraLensPrivate::MapFunctionEnum("tan");
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
  {
    gzwarn << "You should specify lens type using <type> element";
    this->dataPtr->sdf->AddElement("type");
  }

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
    {
      gzerr << "You need a <custom_function> element to use this lens type, "
            << "setting lens type to `stereographic`" << std::endl;

      this->dataPtr->sdf->GetElement("type")->Set("stereographic");
      this->Init(this->Type());
    }
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
      double, std::string> > funTypes = {
    {"gnomonical",      std::make_tuple(1.0, 1.0, 0.0, 1.0, "tan")},
    {"stereographic",   std::make_tuple(2.0, 2.0, 0.0, 1.0, "tan")},
    {"equidistant",     std::make_tuple(1.0, 1.0, 0.0, 1.0, "id")},
    {"equisolid_angle", std::make_tuple(2.0, 2.0, 0.0, 1.0, "sin")},
    {"orthographic",    std::make_tuple(1.0, 1.0, 0.0, 1.0, "sin")}};

  funTypes.emplace("custom",
      std::make_tuple(this->C1(), this->C2(), this->C3(), this->F(),
        CameraLensPrivate::MapFunctionEnum(this->Fun()).AsString()));

  decltype(funTypes)::mapped_type params;

  try
  {
    params = funTypes.at(_type);
  }
  catch(...)
  {
    gzerr << "Unknown lens type [" << _type << "]" << std::endl;
    return;
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

    try
    {
      this->dataPtr->fun =
          CameraLensPrivate::MapFunctionEnum(std::get<4>(params));
    }
    catch(const std::exception &ex)
    {
      gzerr << "`fun` value [" << std::get<4>(params)
            << "] is not known, keeping the old one" << std::endl;
    }
  }
}

//////////////////////////////////////////////////
void CameraLens::SetC1(const double _c)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->dataMutex);

  this->dataPtr->c1 = _c;

  if (!this->IsCustom())
    this->ConvertToCustom();

  this->dataPtr->sdf->GetElement("custom_function")->GetElement("c1")->Set(_c);
}

//////////////////////////////////////////////////
void CameraLens::SetC2(const double _c)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->dataMutex);

  this->dataPtr->c2 = _c;

  if (!this->IsCustom())
    this->ConvertToCustom();

  this->dataPtr->sdf->GetElement("custom_function")->GetElement("c2")->Set(_c);
}

//////////////////////////////////////////////////
void CameraLens::SetC3(const double _c)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->dataMutex);

  this->dataPtr->c3 = _c;

  if (!this->IsCustom())
    this->ConvertToCustom();

  this->dataPtr->sdf->GetElement("custom_function")->GetElement("c3")->Set(_c);
}

//////////////////////////////////////////////////
void CameraLens::SetF(const double _f)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->dataMutex);

  this->dataPtr->f = _f;

  if (!this->IsCustom())
    this->ConvertToCustom();

  this->dataPtr->sdf->GetElement("custom_function")->GetElement("f")->Set(_f);
}

//////////////////////////////////////////////////
void CameraLens::SetFun(const std::string &_fun)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->dataMutex);

  if (!this->IsCustom())
    this->ConvertToCustom();

  try
  {
    this->dataPtr->fun = CameraLensPrivate::MapFunctionEnum(_fun);
  }
  catch(const std::exception &ex)
  {
    gzerr << "`Fun` value [" << _fun << "] is not known, "
          << "keeping the old one" << std::endl;
    return;
  }

  auto customFunction = this->dataPtr->sdf->GetElement("custom_function");
  customFunction->GetElement("fun")->Set(_fun);
}

//////////////////////////////////////////////////
void CameraLens::SetCutOffAngle(const double _angle)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->dataMutex);

  this->dataPtr->cutOffAngle = _angle;

  this->dataPtr->sdf->GetElement("cutoff_angle")->Set(_angle);
}

//////////////////////////////////////////////////
void CameraLens::SetScaleToHFOV(const bool _scale)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->dataMutex);

  this->dataPtr->sdf->GetElement("scale_to_hfov")->Set(_scale);
}

//////////////////////////////////////////////////
void CameraLens::SetUniformVariables(Ogre::Pass *_pass, const float _ratio,
                                     const float _hfov)
{
  Ogre::GpuProgramParametersSharedPtr uniforms =
    _pass->getFragmentProgramParameters();

  uniforms->setNamedConstant("c1", static_cast<Ogre::Real>(this->dataPtr->c1));
  uniforms->setNamedConstant("c2", static_cast<Ogre::Real>(this->dataPtr->c2));
  uniforms->setNamedConstant("c3", static_cast<Ogre::Real>(this->dataPtr->c3));

  if (this->ScaleToHFOV())
  {
    float param = (_hfov/2)/this->dataPtr->c2+this->dataPtr->c3;
    float funRes = this->dataPtr->fun.Apply(static_cast<float>(param));

    float newF = 1.0f/(this->dataPtr->c1*funRes);

    uniforms->setNamedConstant("f", static_cast<Ogre::Real>(newF));
  }
  else
    uniforms->setNamedConstant("f", static_cast<Ogre::Real>(this->dataPtr->f));

  auto vecFun = this->dataPtr->fun.AsVector3d();

  uniforms->setNamedConstant("fun", Ogre::Vector3(
      vecFun.X(), vecFun.Y(), vecFun.Z()));

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
                                 ScenePtr _scene, const bool _autoRender,
                                 const int _textureSize)
    : Camera(_namePrefix, _scene, _autoRender),
      dataPtr(new WideAngleCameraPrivate)
{
  this->dataPtr->lens = new CameraLens();

  this->dataPtr->envCubeMapTexture = NULL;
  this->dataPtr->envTextureSize = _textureSize;

  for (int i = 0; i < 6; ++i)
  {
    this->dataPtr->envCameras[i] = NULL;
    this->dataPtr->envRenderTargets[i] = NULL;
  }
}

//////////////////////////////////////////////////
WideAngleCamera::~WideAngleCamera()
{
  delete this->Lens();
}

//////////////////////////////////////////////////
void WideAngleCamera::Init()
{
  Camera::Init();

  for (int i = 0; i < 6; ++i)
    this->sceneNode->attachObject(this->dataPtr->envCameras[i]);

  // set environment cameras orientation
  this->dataPtr->envCameras[0]->yaw(Ogre::Degree(-90));
  this->dataPtr->envCameras[1]->yaw(Ogre::Degree(90));
  this->dataPtr->envCameras[2]->pitch(Ogre::Degree(90));
  this->dataPtr->envCameras[3]->pitch(Ogre::Degree(-90));
  this->dataPtr->envCameras[5]->yaw(Ogre::Degree(180));

  this->CreateEnvRenderTexture(this->scopedUniqueName + "_envRttTex");
}

//////////////////////////////////////////////////
void WideAngleCamera::Load()
{
  Camera::Load();

  // Cube map texture format defaults to matching image pixel format
  this->dataPtr->envCubeMapTextureFormat =
    static_cast<Ogre::PixelFormat>(this->imageFormat);

  this->CreateEnvCameras();

  if (this->sdf->HasElement("lens"))
  {
    sdf::ElementPtr sdfLens = this->sdf->GetElement("lens");

    this->dataPtr->lens->Load(sdfLens);

    if (sdfLens->HasElement("env_texture_size"))
      this->dataPtr->envTextureSize = sdfLens->Get<int>("env_texture_size");

    const std::string envTextureFormat = "ignition:env_texture_format";
    if (sdfLens->HasElement(envTextureFormat))
    {
      this->dataPtr->envCubeMapTextureFormat = static_cast<Ogre::PixelFormat>(
        this->OgrePixelFormat(sdfLens->Get<std::string>(envTextureFormat)));
    }
  }
  else
    this->dataPtr->lens->Load();

  std::string lensType = this->dataPtr->lens->Type();
  if (lensType == "gnomonical" && this->HFOV() > (IGN_PI/2.0))
  {
    gzerr << "The recommended camera horizontal FOV should be <= PI/2"
        << " for lens of type 'gnomonical'." << std::endl;
  }
}

//////////////////////////////////////////////////
void WideAngleCamera::Fini()
{
  for (int i = 0; i < 6; ++i)
  {
    RTShaderSystem::DetachViewport(this->dataPtr->envViewports[i],
                                   this->GetScene());

    this->dataPtr->envRenderTargets[i]->removeAllViewports();
    this->dataPtr->envRenderTargets[i] = NULL;

    this->GetScene()->OgreSceneManager()->destroyCamera(
        this->dataPtr->envCameras[i]->getName());
    this->dataPtr->envCameras[i] = NULL;
  }

  if (this->dataPtr->envCubeMapTexture)
    Ogre::TextureManager::getSingleton().remove(
      this->dataPtr->envCubeMapTexture->getName());

  this->dataPtr->envCubeMapTexture = NULL;

  Camera::Fini();
}

//////////////////////////////////////////////////
int WideAngleCamera::EnvTextureSize() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->dataMutex);

  return this->dataPtr->envTextureSize;
}

//////////////////////////////////////////////////
CameraLens *WideAngleCamera::Lens() const
{
  return this->dataPtr->lens;
}

//////////////////////////////////////////////////
void WideAngleCamera::SetRenderTarget(Ogre::RenderTarget *_target)
{
  Camera::SetRenderTarget(_target);

  if (this->renderTarget)
  {
    this->dataPtr->cubeMapCompInstance =
      Ogre::CompositorManager::getSingleton().addCompositor(this->viewport,
          "WideCameraLensMap/ParametrisedMap");

    if (this->dataPtr->envCubeMapTexture)
    {
      this->dataPtr->compMat =
          Ogre::MaterialManager::getSingleton().getByName("Gazebo/WideLensMap");

      auto pass = this->dataPtr->compMat->getTechnique(0)->getPass(0);

      if (!pass->getNumTextureUnitStates())
        pass->createTextureUnitState();

      this->dataPtr->cubeMapCompInstance->addListener(this);
    }
    else
      gzerr << "Compositor texture MISSING" << std::endl;


    this->dataPtr->cubeMapCompInstance->setEnabled(true);
  }
}

//////////////////////////////////////////////////
void WideAngleCamera::SetEnvTextureSize(const int _size)
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

    this->dataPtr->envCameras[i] =
        this->GetScene()->OgreSceneManager()->createCamera(name_str.str());

    this->dataPtr->envCameras[i]->setFixedYawAxis(false);
    this->dataPtr->envCameras[i]->setFOVy(Ogre::Degree(90));
    this->dataPtr->envCameras[i]->setAspectRatio(1);

    this->dataPtr->envCameras[i]->yaw(Ogre::Degree(-90.0));
    this->dataPtr->envCameras[i]->roll(Ogre::Degree(-90.0));

    this->dataPtr->envCameras[i]->setNearClipDistance(0.01);
    this->dataPtr->envCameras[i]->setFarClipDistance(1000);
  }
}

//////////////////////////////////////////////////
void WideAngleCamera::SetClipDist()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->dataMutex);

  // <clip> element presence is already checked
  // in Camera::SetClipDist(float,float)
  sdf::ElementPtr clipElem = this->sdf->GetElement("clip");

  for (int i = 0; i < 6; ++i)
  {
    if (this->dataPtr->envCameras[i])
    {
      this->dataPtr->envCameras[i]->setNearClipDistance(
          clipElem->Get<double>("near"));
      this->dataPtr->envCameras[i]->setFarClipDistance(
          clipElem->Get<double>("far"));
      this->dataPtr->envCameras[i]->setRenderingDistance(
          clipElem->Get<double>("far"));
    }
    else
    {
      gzerr << "Setting clip distances failed - no camera yet" << std::endl;

      break;
    }
  }
}

//////////////////////////////////////////////////
bool WideAngleCamera::SetBackgroundColor(const ignition::math::Color &_color)
{
  bool retVal = true;
  Ogre::ColourValue clr = Conversions::Convert(_color);
  if (this->OgreViewport())
  {
    this->OgreViewport()->setBackgroundColour(clr);
    for (int i = 0; i < 6; ++i)
    {
      if (this->dataPtr->envViewports[i])
      {
        this->dataPtr->envViewports[i]->setBackgroundColour(clr);
      }
      else
      {
        retVal = false;
      }
    }
  }
  else
  {
    retVal = false;
  }
  return retVal;
}

//////////////////////////////////////////////////
void WideAngleCamera::CreateEnvRenderTexture(const std::string &_textureName)
{
  unsigned int fsaa = 0;
  std::vector<unsigned int> fsaaLevels =
      RenderEngine::Instance()->FSAALevels();
  // check if target fsaa is supported
  unsigned int targetFSAA = 4;
  auto const it = std::find(fsaaLevels.begin(), fsaaLevels.end(), targetFSAA);
  if (it != fsaaLevels.end())
    fsaa = targetFSAA;

  this->dataPtr->envCubeMapTexture =
      Ogre::TextureManager::getSingleton().createManual(
          this->scopedUniqueName+"::"+_textureName,
          "General",
          Ogre::TEX_TYPE_CUBE_MAP,
          this->dataPtr->envTextureSize,
          this->dataPtr->envTextureSize,
          0,
          this->dataPtr->envCubeMapTextureFormat,
          Ogre::TU_RENDERTARGET,
          0,
          false,
          fsaa).getPointer();

  for (int i = 0; i < 6; ++i)
  {
    Ogre::RenderTarget *rtt;
    rtt = this->dataPtr->envCubeMapTexture->getBuffer(i)->getRenderTarget();
    rtt->setAutoUpdated(false);

    Ogre::Viewport *vp = rtt->addViewport(this->dataPtr->envCameras[i]);
    vp->setClearEveryFrame(true);
    vp->setShadowsEnabled(true);
    vp->setOverlaysEnabled(false);

    RTShaderSystem::AttachViewport(vp, this->GetScene());

    auto const &gzBgColor = this->scene->BackgroundColor();
    vp->setBackgroundColour(Conversions::Convert(gzBgColor));
    vp->setVisibilityMask(GZ_VISIBILITY_ALL &
        ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE));

    this->dataPtr->envViewports[i] = vp;

    if (this->GetScene()->GetSkyX())
      rtt->addListener(this->GetScene()->GetSkyX());

    this->dataPtr->envRenderTargets[i] = rtt;
  }
}

//////////////////////////////////////////////////
void WideAngleCamera::RenderImpl()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->renderMutex);

  for (int i = 0; i < 6; ++i)
    this->dataPtr->envRenderTargets[i]->update();

  this->dataPtr->compMat->getTechnique(0)->getPass(0)->getTextureUnitState(0)->
      setTextureName(this->dataPtr->envCubeMapTexture->getName());

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
    gzerr << "No lens" << std::endl;
    return;
  }

  this->Lens()->SetUniformVariables(pPass,
    this->AspectRatio(),
    this->HFOV().Radian());

#if defined(HAVE_OPENGL)
  // XXX: OGRE doesn't allow to enable cubemap filtering extention thru its API
  // suppose that this function was invoked in a thread that has OpenGL context
  glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);
#endif
}

//////////////////////////////////////////////////
void WideAngleCamera::UpdateFOV()
{
  // override to prevent parent class from updating fov as
  // it'll be handled here in this class.
}

//////////////////////////////////////////////////
ignition::math::Vector3d WideAngleCamera::Project3d(
    const ignition::math::Vector3d &_pt) const
{
  // project onto cubemap face then onto
  ignition::math::Vector3d screenPos;
  // loop through all env cameras can find the one that sees the 3d world point
  for (int i = 0; i < 6; ++i)
  {
    // project world point to camera clip space.
    auto viewProj = this->dataPtr->envCameras[i]->getProjectionMatrix() *
        this->dataPtr->envCameras[i]->getViewMatrix();
    auto pos = viewProj * Ogre::Vector4(Conversions::Convert(_pt));
    pos.x /= pos.w;
    pos.y /= pos.w;
    // check if point is visible
    if (std::fabs(pos.x) <= 1 && std::fabs(pos.y) <= 1 && pos.z > 0)
    {
      // determine dir vector to projected point from env camera
      // work in y up, z forward, x right clip space
      ignition::math::Vector3d dir(pos.x, pos.y, 1);
      ignition::math::Quaterniond rot = ignition::math::Quaterniond::Identity;

      // rotate dir vector into wide angle camera frame based on the
      // face of the cube. Note: operate in clip space so
      // left handed coordinate system rotation
      if (i == 0)
        rot = ignition::math::Quaterniond(0.0, M_PI*0.5, 0.0);
      else if (i == 1)
        rot = ignition::math::Quaterniond(0.0, -M_PI*0.5, 0.0);
      else if (i == 2)
        rot = ignition::math::Quaterniond(-M_PI*0.5, 0.0, 0.0);
      else if (i == 3)
        rot = ignition::math::Quaterniond(M_PI*0.5, 0.0, 0.0);
      else if (i == 5)
        rot = ignition::math::Quaterniond(0.0, M_PI, 0.0);
      dir = rot * dir;
      dir.Normalize();

      // compute theta and phi from the dir vector
      // theta is angle to dir vector from z (forward)
      // phi is angle from x in x-y plane
      // direction vector (x, y, z)
      // x = sin(theta)cos(phi)
      // y = sin(theta)sin(phi)
      // z = cos(theta)
      double theta =  std::atan2(
          std::sqrt(dir.X() * dir.X() + dir.Y() * dir.Y()), dir.Z());
      double phi = std::atan2(dir.Y(), dir.X());
      // this also works:
      // double theta = std::acos(dir.Z());
      // double phi = std::asin(dir.Y() / std::sin(theta));

      double f = this->Lens()->F();
      double hfov = this->HFOV().Radian();
      // recompute f if scale to HFOV is true
      if (this->Lens()->ScaleToHFOV())
      {
        double param = (hfov/2.0) / this->Lens()->C2() + this->Lens()->C3();
        double funRes =
            CameraLensPrivate::MapFunctionEnum(this->Lens()->Fun()).Apply(
            static_cast<float>(param));
        f = 1.0/(this->Lens()->C1()*funRes);
      }

      // Apply fisheye lens mapping function
      // r is distance of point from image center
      double r = this->Lens()->C1() * f *
          CameraLensPrivate::MapFunctionEnum(this->Lens()->Fun()).Apply(
          theta/this->Lens()->C2() + this->Lens()->C3());

      // compute projected x and y in clip space
      double x = cos(phi) * r;
      double y = sin(phi) * r;

      // env cam cube map texture is square and likely to be different size from
      // viewport. We need to adjust projected pos based on aspect ratio
      double aspect = static_cast<double>(this->ViewportWidth()) /
        static_cast<double>(this->ViewportHeight());
      y *= aspect;

      // convert to screen space
      screenPos.X() = ((x / 2.0) + 0.5) * this->ViewportWidth();
      screenPos.Y() = (1 - ((y / 2.0) + 0.5)) * this->ViewportHeight();

      // r will be > 1.0 if point is not visible (outside of image)
      screenPos.Z() = r;
      return screenPos;
    }
  }

  return screenPos;
}

//////////////////////////////////////////////////
std::vector<Ogre::Camera *> WideAngleCamera::OgreEnvCameras() const
{
  return std::vector<Ogre::Camera *>(
    std::begin(this->dataPtr->envCameras), std::end(this->dataPtr->envCameras));
}
