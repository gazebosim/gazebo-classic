/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#ifndef GAZEBO_OGRE_H_
#define GAZEBO_OGRE_H_

// This disables warning messages for OGRE
#pragma GCC system_header

// This prevents some deprecation #warning messages on OSX 10.9
#pragma clang diagnostic ignored "-W#warnings"

#include <OGRE/Ogre.h>
#include <OGRE/OgreBillboard.h>
#include <OGRE/OgreBitwise.h>
#include <OGRE/OgreImageCodec.h>
#include <OGRE/OgreMovableObject.h>
#include <OGRE/OgreRenderable.h>
#include <OGRE/OgrePlugin.h>
#include <OGRE/OgreDataStream.h>
#include <OGRE/OgreLogManager.h>
#include <OGRE/OgreSceneQuery.h>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreMatrix4.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreMesh.h>
#include <OGRE/OgreHardwareBufferManager.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreNode.h>
#include <OGRE/OgreSimpleRenderable.h>
#include <OGRE/OgreFrameListener.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreRenderObjectListener.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgrePass.h>
#include <OGRE/OgreTextureUnitState.h>
#include <OGRE/OgreGpuProgramManager.h>
#include <OGRE/OgreHighLevelGpuProgramManager.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OGRE/OgreShadowCameraSetupPSSM.h>
#include <OGRE/Paging/OgrePageManager.h>
#include <OGRE/Paging/OgrePagedWorld.h>
#include <OGRE/Terrain/OgreTerrainPaging.h>
#include <OGRE/Terrain/OgreTerrainMaterialGeneratorA.h>
#include <OGRE/Terrain/OgreTerrain.h>
#include <OGRE/Terrain/OgreTerrainGroup.h>

#if OGRE_VERSION_MAJOR > 1 || OGRE_VERSION_MINOR >= 7
#include <OGRE/RTShaderSystem/OgreRTShaderSystem.h>
#include <OGRE/RTShaderSystem/OgreShaderProgramSet.h>
#include <OGRE/RTShaderSystem/OgreShaderGLSLProgramWriter.h>
#include <OGRE/RTShaderSystem/OgreShaderProgramWriterManager.h>
#include <OGRE/RTShaderSystem/OgreShaderFunction.h>
#include <OGRE/RTShaderSystem/OgreShaderProgram.h>
#endif

#if OGRE_VERSION_MAJOR > 1 || OGRE_VERSION_MINOR >= 9
#include <OGRE/Overlay/OgreOverlayManager.h>
#include <OGRE/Overlay/OgreOverlayElement.h>
#include <OGRE/Overlay/OgreOverlayContainer.h>
#include <OGRE/Overlay/OgreFontManager.h>
#include <OGRE/Overlay/OgreOverlaySystem.h>
#else
#include <OGRE/OgreFontManager.h>
#endif

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR < 11
// The  <OGRE/OgreWindowEventUtilities.h> header has always been included in
// public headers for Gazebo <= 10, but was moved to the Bites component in
// Ogre 1.11 (see  https://github.com/OGRECave/ogre/pull/647). As it is not
// used at all in Gazebo, we can just include it for Ogre <= 1.10 to avoid
// breaking transitive includes in downstream projects.
// In Gazebo 11, this can be removed.
#include <OGRE/OgreWindowEventUtilities.h>
#endif

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 11
#define GZ_OGRE_SET_MATERIAL_BY_NAME(ptr, name) \
  (ptr)->setMaterial(Ogre::MaterialManager::getSingleton().getByName(name))
#else
#define GZ_OGRE_SET_MATERIAL_BY_NAME(ptr, name) \
  (ptr)->setMaterial(name)
#endif

#endif
