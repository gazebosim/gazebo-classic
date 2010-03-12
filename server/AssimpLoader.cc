#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "Mesh.hh"
#include "Material.hh"
#include "AssimpLoader.hh"
#include <sys/stat.h>

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
AssimpLoader::AssimpLoader()
  : MeshLoader()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
AssimpLoader::~AssimpLoader()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load
Mesh *AssimpLoader::Load(const std::string &filename)
{
  const aiScene *scene;

  scene = this->importer.ReadFile( filename.c_str(),
                                   //aiProcess_CalcTangentSpace |
                                   //aiProcess_Triangulate |
                                   //aiProcess_JoinIdenticalVertices |
                                   aiProcess_SortByPType );

  if (!scene)
  {
    gzerr(0) << "Importer Error:" << this->importer.GetErrorString();
    gzthrow("Unable to import mesh file[" << filename << "] using assimp");
  }

  Mesh *mesh = new Mesh();

  for (unsigned int i=0; i < scene->mNumMaterials; i++)
  {
    Material *mat = new Material();
    aiMaterial *amat = scene->mMaterials[i];

    for (unsigned int j=0; j < amat->mNumProperties; j++)
    {
      aiMaterialProperty *prop = amat->mProperties[j];
      std::string propKey = prop->mKey.data;

      if (propKey == "$tex.file")
      {
        aiString texName;
        aiTextureMapping mapping;
        unsigned int uvIndex;
        amat->GetTexture(aiTextureType_DIFFUSE,0, &texName, &mapping, &uvIndex);

        // attempt to find fully qualified filename for texture image
        // in the same place as the collada file
        // pathname of the collada dae file
        std::string dae_pathname = filename.rfind("/")==std::string::npos ?
                  std::string("") : filename.substr(0,filename.rfind("/")+1);
        // fully qualified texture filename
        std::string texture_fqfn = dae_pathname + texName.data;
        struct stat st;
        if (stat(texture_fqfn.c_str(), &st) == 0)
        {
          mat->SetTextureImage(texName.data,dae_pathname); // use fqfn if it exists
        }
        else
          mat->SetTextureImage(texName.data); // use defaut filename, rely on ogre path
      }
      else if (propKey == "?mat.name")
      {
        aiString matName;
        amat->Get(AI_MATKEY_NAME, matName);
        std::string uniqueMatName = filename + matName.data;
        mat->SetName(uniqueMatName);
      }
      else if (propKey == "$clr.diffuse")
      {
        aiColor3D clr;
        amat->Get(AI_MATKEY_COLOR_DIFFUSE, clr);
        mat->SetDiffuse( Color(clr.r, clr.g, clr.b) );
      }
      else if (propKey == "$clr.ambient")
      {
        aiColor3D clr;
        amat->Get(AI_MATKEY_COLOR_AMBIENT, clr);
        mat->SetAmbient( Color(clr.r, clr.g, clr.b) );
      }
      else if (propKey == "$clr.specular")
      {
        aiColor3D clr;
        amat->Get(AI_MATKEY_COLOR_SPECULAR, clr);
        mat->SetSpecular( Color(clr.r, clr.g, clr.b) );
      }
      else if (propKey == "$clr.emissive")
      {
        aiColor3D clr;
        amat->Get(AI_MATKEY_COLOR_EMISSIVE, clr);
        mat->SetEmissive( Color(clr.r, clr.g, clr.b) );
      }
      else if (propKey == "$clr.opacity")
      {
        float o;
        amat->Get(AI_MATKEY_OPACITY, o);
        mat->SetTransparency(o);
      }
      else if (propKey == "$mat.shininess")
      {
        float s;
        amat->Get(AI_MATKEY_SHININESS, s);
        mat->SetShininess(s);
      }
      else if (propKey == "$mat.blend")
      {
        int mode;
        amat->Get(AI_MATKEY_BLEND_FUNC, mode);
        switch(mode)
        {
          case aiBlendMode_Additive:
            mat->SetBlendMode(Material::ADD);
            break;
          case aiBlendMode_Default:
          default:
            mat->SetBlendMode(Material::REPLACE);
            break;
        }
      }
      else if (propKey == "$mat.shadingm")
      {
        int model;
        amat->Get(AI_MATKEY_SHADING_MODEL, model);
        switch(model)
        {
          case aiShadingMode_Flat:
            mat->SetShadeMode(Material::FLAT);
            break;
          case aiShadingMode_Phong:
            mat->SetShadeMode(Material::PHONG);
            break;
          case aiShadingMode_Gouraud:
          default:
            mat->SetShadeMode(Material::GOURAUD);
            break;
        }
      }
    }

    mesh->AddMaterial(mat);
  }

  this->BuildMesh(scene->mRootNode, mesh);

  return mesh;
}

////////////////////////////////////////////////////////////////////////////////
// Build a mesh
void AssimpLoader::BuildMesh(aiNode *node, Mesh *mesh)
{
  if (node == NULL)
    return;

  unsigned int i,j,k;
  SubMesh *subMesh = NULL;
  const aiScene *scene  = this->importer.GetScene();

  aiMatrix4x4 transform = node->mTransformation;

  aiNode *pnode = node->mParent;

  while (pnode)
  {
    // Don't convert to y-up orientation, which is what the root node in
    // Assimp does
    if (pnode->mParent != NULL)
      transform = pnode->mTransformation * transform;
    pnode = pnode->mParent;
  }

  // Add each mesh
  for (i=0; i < node->mNumMeshes; i++)
  {
    aiVector3D p;

    subMesh = new SubMesh();

    aiMesh *aMesh = scene->mMeshes[i];

    subMesh->SetMaterialIndex( aMesh->mMaterialIndex );

    // Add in the indices for each face
    for (j=0; j < aMesh->mNumFaces; j++)
    {
      aiFace *aFace = &(aMesh->mFaces[j]);

      for (k=0; k < aFace->mNumIndices; k++)
        subMesh->AddIndex( aFace->mIndices[k]);
    }

    // Add in the vertices
    for (j = 0; j < aMesh->mNumVertices; j++)
    {
      p.x = aMesh->mVertices[j].x;
      p.y = aMesh->mVertices[j].y;
      p.z = aMesh->mVertices[j].z;

      p *= transform;

      subMesh->AddVertex(p.x, p.y, p.z); 

      if (aMesh->HasNormals())
      {
        p.x = aMesh->mNormals[j].x;
        p.y = aMesh->mNormals[j].y;
        p.z = aMesh->mNormals[j].z;
      }

      subMesh->AddNormal(p.x, p.y, p.z); 
      if (aMesh->mNumUVComponents[0])
        subMesh->AddTexCoord(aMesh->mTextureCoords[0][j].x, 
                             1.0-aMesh->mTextureCoords[0][j].y);
      else
      {
        // Auto-generate projected texture coordinates, projected from center of aabb
        // @todo: implement globally for all meshes as an option, or move this to Material.cc
        double x = p.x;
        double y = p.y;
        double z = p.z;
        double r = std::max(0.000001,sqrt(x*x+y*y+z*z));
        double s = std::min(1.0,std::max(-1.0,z/r));
        double t = std::min(1.0,std::max(-1.0,y/r));
        double u = acos(s) / M_PI;
        double v = acos(t) / M_PI;
        //std::cerr << "uv1 debug: " << u << "," << v << std::endl;
        subMesh->AddTexCoord(u,v);
      }
    }

    mesh->AddSubMesh(subMesh);
  }

  for (i=0; i < node->mNumChildren; i++)
  {
    this->BuildMesh( node->mChildren[i], mesh );
  }
}
