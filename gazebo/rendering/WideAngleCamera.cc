//
// Created by klokik on 02.07.15.
//

#include "WideAngleCamera.hh"

#include "gazebo/rendering/ogre_gazebo.h"


using namespace gazebo;
using namespace rendering;

WideAngleCamera::WideAngleCamera(const std::string &_namePrefix, ScenePtr _scene, bool _autoRender):
  Camera(_namePrefix,_scene,_autoRender)
{
}

WideAngleCamera::~WideAngleCamera()
{
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

    this->wamapInstance->setEnabled(true);
  }
}