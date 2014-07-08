/*
 * Copyright 2014 Open Source Robotics Foundation
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

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Mesh.hh"

#include "gazebo/physics/dart/DARTCollision.hh"
#include "gazebo/physics/dart/DARTMeshShape.hh"
#include "gazebo/physics/dart/DARTPhysics.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
// Constructor of aiScene is missing so we define it here.
aiScene::aiScene()
{
  mFlags = 0;
  mRootNode = NULL;
  mNumMeshes = 0;
  mMeshes = NULL;
  mNumMaterials = 0;
  mMaterials = NULL;
  mNumAnimations = 0;
  mAnimations = NULL;
  mNumTextures = 0;
  mTextures = NULL;
  mNumLights = 0;
  mLights = NULL;
  mNumCameras = 0;
  mCameras = NULL;
}

//////////////////////////////////////////////////
DARTMeshShape::DARTMeshShape(CollisionPtr _parent) : MeshShape(_parent)
{
}

//////////////////////////////////////////////////
DARTMeshShape::~DARTMeshShape()
{
}

//////////////////////////////////////////////////
void DARTMeshShape::Update()
{
  MeshShape::Update();
}

//////////////////////////////////////////////////
void DARTMeshShape::Load(sdf::ElementPtr _sdf)
{
  MeshShape::Load(_sdf);
}

//////////////////////////////////////////////////
void DARTMeshShape::Init()
{
  MeshShape::Init();

  DARTCollisionPtr dartCollisionParent
      = boost::dynamic_pointer_cast<DARTCollision>(this->collisionParent);
  GZ_ASSERT(dartCollisionParent, "dartCollisionParent is null");

  float *localVertices = NULL;
  int   *localIndices  = NULL;

  unsigned int numVertices = this->mesh->GetVertexCount();
  unsigned int numIndices  = this->mesh->GetIndexCount();

  // Get all the vertex and index data
  this->mesh->FillArrays(&localVertices, &localIndices);

  // Scale
  math::Vector3 meshScale = this->sdf->Get<math::Vector3>("scale");

  // Create new aiScene (aiMesh)
  aiScene *assimpScene = new aiScene;
  aiMesh *assimpMesh   = new aiMesh;
  assimpScene->mNumMeshes = 1;
  assimpScene->mMeshes    = new aiMesh*[1];
  assimpScene->mMeshes[0] = assimpMesh;

  // Set localVertices and normals
  assimpMesh->mNumVertices = numVertices;
  assimpMesh->mVertices    = new aiVector3D[numVertices];
  assimpMesh->mNormals     = new aiVector3D[numVertices];
  aiVector3D itAIVector3d;
  for (unsigned int i = 0; i < numVertices; ++i)
  {
    itAIVector3d.Set(localVertices[i*3 + 0], localVertices[i*3 + 1],
      localVertices[i*3 + 2]);
    assimpMesh->mVertices[i] = itAIVector3d;
    assimpMesh->mNormals[i]  = itAIVector3d;
  }

  // Set faces
  assimpMesh->mNumFaces = numIndices/3;
  assimpMesh->mFaces    = new aiFace[assimpMesh->mNumFaces];
  for (unsigned int i = 0; i < assimpMesh->mNumFaces; ++i)
  {
    aiFace *itAIFace = &assimpMesh->mFaces[i];
    itAIFace->mNumIndices = 3;
    itAIFace->mIndices    = new unsigned int[3];
    itAIFace->mIndices[0] = localIndices[i*3 + 0];
    itAIFace->mIndices[1] = localIndices[i*3 + 1];
    itAIFace->mIndices[2] = localIndices[i*3 + 2];
  }

  dart::dynamics::MeshShape *dtMeshShape
      = new dart::dynamics::MeshShape(
        DARTTypes::ConvVec3(meshScale), assimpScene);
  GZ_ASSERT(dartCollisionParent->GetDARTBodyNode(),
    "dartCollisionParent->GetDARTBodyNode() is null");
  dartCollisionParent->GetDARTBodyNode()->addCollisionShape(dtMeshShape);
  dartCollisionParent->SetDARTCollisionShape(dtMeshShape);

  delete [] localVertices;
  delete [] localIndices;
}
