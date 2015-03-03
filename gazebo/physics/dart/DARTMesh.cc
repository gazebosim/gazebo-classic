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
#include "gazebo/common/Mesh.hh"

#include "gazebo/physics/dart/DARTCollision.hh"
#include "gazebo/physics/dart/DARTPhysics.hh"
#include "gazebo/physics/dart/DARTMesh.hh"

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
DARTMesh::DARTMesh()
{
}

//////////////////////////////////////////////////
DARTMesh::~DARTMesh()
{
}

//////////////////////////////////////////////////
void DARTMesh::Init(const common::SubMesh *_subMesh,
                    DARTCollisionPtr _collision,
                    const math::Vector3 &_scale)
{
  float *vertices = NULL;
  int *indices = NULL;

  unsigned int numVertices = _subMesh->GetVertexCount();
  unsigned int numIndices = _subMesh->GetIndexCount();

  // Get all the vertex and index data
  _subMesh->FillArrays(&vertices, &indices);

  this->CreateMesh(vertices, indices, numVertices, numIndices,
                   _collision, _scale);

  delete [] vertices;
  delete [] indices;
}

//////////////////////////////////////////////////
void DARTMesh::Init(const common::Mesh *_mesh,
                    DARTCollisionPtr _collision,
                    const math::Vector3 &_scale)
{
  float *vertices = NULL;
  int *indices = NULL;

  unsigned int numVertices = _mesh->GetVertexCount();
  unsigned int numIndices = _mesh->GetIndexCount();

  // Get all the vertex and index data
  _mesh->FillArrays(&vertices, &indices);

  this->CreateMesh(vertices, indices, numVertices, numIndices,
                   _collision, _scale);
  delete [] vertices;
  delete [] indices;
}

/////////////////////////////////////////////////
void DARTMesh::CreateMesh(float *_vertices, int *_indices,
    unsigned int _numVertices, unsigned int _numIndices,
    DARTCollisionPtr _collision, const math::Vector3 &_scale)
{
  GZ_ASSERT(_collision, "DART collision is null");

  // Create new aiScene (aiMesh)
  aiScene *assimpScene = new aiScene;
  aiMesh *assimpMesh = new aiMesh;
  assimpScene->mNumMeshes = 1;
  assimpScene->mMeshes = new aiMesh*[1];
  assimpScene->mMeshes[0] = assimpMesh;

  // Set _vertices and normals
  assimpMesh->mNumVertices = _numVertices;
  assimpMesh->mVertices = new aiVector3D[_numVertices];
  assimpMesh->mNormals = new aiVector3D[_numVertices];
  aiVector3D itAIVector3d;

  for (unsigned int i = 0; i < _numVertices; ++i)
  {
    itAIVector3d.Set(_vertices[i*3 + 0], _vertices[i*3 + 1],
      _vertices[i*3 + 2]);
    assimpMesh->mVertices[i] = itAIVector3d;
    assimpMesh->mNormals[i]  = itAIVector3d;
  }

  // Set faces
  assimpMesh->mNumFaces = _numIndices/3;
  assimpMesh->mFaces = new aiFace[assimpMesh->mNumFaces];
  for (unsigned int i = 0; i < assimpMesh->mNumFaces; ++i)
  {
    aiFace* itAIFace = &assimpMesh->mFaces[i];
    itAIFace->mNumIndices = 3;
    itAIFace->mIndices = new unsigned int[3];
    itAIFace->mIndices[0] = _indices[i*3 + 0];
    itAIFace->mIndices[1] = _indices[i*3 + 1];
    itAIFace->mIndices[2] = _indices[i*3 + 2];
  }

  dart::dynamics::MeshShape *dtMeshShape = new dart::dynamics::MeshShape(
      DARTTypes::ConvVec3(_scale), assimpScene);
  GZ_ASSERT(_collision->GetDARTBodyNode(),
    "DART _collision->GetDARTBodyNode() is null");
  _collision->GetDARTBodyNode()->addCollisionShape(dtMeshShape);
  _collision->SetDARTCollisionShape(dtMeshShape);
}
