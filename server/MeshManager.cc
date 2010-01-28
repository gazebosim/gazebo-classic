#include <sys/stat.h>
#include <string>

#include "Simulator.hh"
#include "GazeboConfig.hh"
#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "Mesh.hh"
#include "OgreLoader.hh"
#include "AssimpLoader.hh"
#include "STLLoader.hh"

#include "MeshManager.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
MeshManager::MeshManager()
{
  this->assimpLoader = new AssimpLoader();
  this->ogreLoader = new OgreLoader();
  this->stlLoader = new STLLoader();

  // Create some basic shapes
  this->CreateSphere("unit_sphere",1.0, 32, 32);
  this->CreateSphere("joint_anchor",0.01, 32, 32);
  this->CreateBox("body_cg", Vector3(0.014,0.014,0.014), 
                             Vector2<double>(0.014,0.014));
  this->CreateBox("unit_box_U1V1", Vector3(1,1,1), 
                             Vector2<double>(1,1));
  this->CreateCylinder("unit_cylinder", 0.5, 1.0, 1, 32);
  this->CreateCone("unit_cone", 0.5, 1.0, 5, 32);
  this->CreateCamera("unit_camera", 0.5);
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
MeshManager::~MeshManager()
{
  delete this->assimpLoader;
  delete this->ogreLoader;
  delete this->stlLoader;
}

////////////////////////////////////////////////////////////////////////////////
// Load a mesh from a file
const Mesh *MeshManager::Load(const std::string &filename)
{
  struct stat st;
  std::string fullname;
  bool found = false;
  Mesh *mesh = NULL;

  std::string extension;

  if (this->HasMesh(filename))
  {
    //return this->meshes[filename];
    
    // erase mesh from this->meshes. This allows a mesh to be modified and
    // inserted into gazebo again without closing gazebo.
    std::map<std::string, Mesh*>::iterator iter;
    iter = this->meshes.find(filename);
    delete iter->second;
    iter->second = NULL;
    this->meshes.erase(iter);
  }

  fullname =  std::string("./")+filename;
  if (stat(fullname.c_str(), &st) == 0)
  {
    found = true;
  }
  else if ( stat(filename.c_str(), &st) == 0)
  {
    fullname =  filename;
    found = true;
  }
  else
  {
    std::list<std::string> gazeboPaths;
    gazeboPaths = Simulator::Instance()->GetGazeboConfig()->GetGazeboPaths();
    for (std::list<std::string>::iterator iter=gazeboPaths.begin(); 
        iter!=gazeboPaths.end(); ++iter)
    {
      fullname = (*iter)+"/Media/models/"+filename;
      if (stat(fullname.c_str(), &st) == 0)
      {
        found = true;
        break;
      }
    }
  }

  if (found)
  {
    extension = fullname.substr(fullname.rfind(".")+1, fullname.size());
    MeshLoader *loader = NULL;

    if (extension == "mesh")
      loader = this->ogreLoader;
    else if (extension == "stl" || extension == "stlb" || extension == "stla")
      loader= this->stlLoader;
    else
      loader = this->assimpLoader;

    try 
    {
      if (!this->HasMesh(filename))
      {
        mesh = loader->Load(fullname);
        mesh->SetName(filename);
        this->meshes.insert( std::make_pair(filename, mesh) );
      }
      else
      {
        mesh = this->meshes[filename];
      }
    } 
    catch (gazebo::GazeboError e)
    {
      gzerr(0) << "Error loading mesh[" << fullname << "]\n";
      gzerr(0) << e << "\n";
      gzthrow(e);
    }
  }
  else
    gzerr(0) << "Unable to find file[" << filename << "]\n";

  mesh->RecalculateNormals();
  return mesh;
}

////////////////////////////////////////////////////////////////////////////////
/// Add a mesh to the manager
void MeshManager::AddMesh(Mesh *mesh)
{
  if (!this->HasMesh(mesh->GetName()))
    this->meshes[mesh->GetName()] = mesh;
}

////////////////////////////////////////////////////////////////////////////////
/// Get a mesh by name
const Mesh *MeshManager::GetMesh(const std::string &name) const
{
  std::map<std::string, Mesh*>::const_iterator iter;

  iter = this->meshes.find(name);

  if (iter != this->meshes.end())
    return iter->second;

  gzerr(0) << "Unable to find mesh with name[" << name << "]\n";
  return NULL;
}
 
////////////////////////////////////////////////////////////////////////////////
/// Return true if the mesh exists
bool MeshManager::HasMesh(const std::string &name) const
{
  std::map<std::string, Mesh*>::const_iterator iter;
  iter = this->meshes.find(name);

  return iter != this->meshes.end();
}

////////////////////////////////////////////////////////////////////////////////
// Create a sphere
void MeshManager::CreateSphere(const std::string &name, float radius, 
                               int rings, int segments)
{
  if (this->HasMesh(name))
  {
    return;
  }

  int ring, seg;
  float r0;
  float deltaSegAngle = (2.0 * M_PI / segments);
  float deltaRingAngle = (M_PI / rings);
  Vector3 vert, norm;
  unsigned short verticeIndex = 0;

  Mesh *mesh = new Mesh();
  mesh->SetName(name);
  this->meshes.insert( std::make_pair(name, mesh));

  SubMesh *subMesh = new SubMesh();
  mesh->AddSubMesh(subMesh);

  // Generate the group of rings for the sphere
  for (ring = 0; ring <= rings; ring++)
  {
    r0 = radius * sinf (ring * deltaRingAngle);
    vert.y = radius * cosf (ring * deltaRingAngle);

    // Generate the group of segments for the current ring
    for (seg = 0; seg <= segments; seg++)
    {
      vert.x = r0 * sinf(seg * deltaSegAngle);
      vert.z = r0 * cosf(seg * deltaSegAngle);

      // TODO: Don't think these normals are correct.
      norm = vert;
      norm.Normalize();

      // Add one vertex to the strip which makes up the sphere
      subMesh->AddVertex(vert);
      subMesh->AddNormal(norm);
      subMesh->AddTexCoord((float) seg / (float) segments, 
                        (float) ring / (float) rings );

      if (ring != rings)
      {
        // each vertex (except the last) has six indices pointing to it
        subMesh->AddIndex(verticeIndex + segments + 1);
        subMesh->AddIndex(verticeIndex);
        subMesh->AddIndex(verticeIndex + segments);
        subMesh->AddIndex(verticeIndex + segments + 1);
        subMesh->AddIndex(verticeIndex + 1);
        subMesh->AddIndex(verticeIndex);

        verticeIndex++;
      }
    }
  }

  mesh->RecalculateNormals();
}


////////////////////////////////////////////////////////////////////////////////
/// Create a Box mesh
void MeshManager::CreateBox(const std::string &name, const Vector3 &sides,
                            const Vector2<double> &uvCoords)
{
  int i,k;

  if (this->HasMesh(name))
  {
    return;
  }

  Mesh *mesh = new Mesh();
  mesh->SetName(name);
  this->meshes.insert( std::make_pair(name, mesh) );

  SubMesh *subMesh = new SubMesh();
  mesh->AddSubMesh(subMesh);

  // Vertex values
  float v[8][3] =
  {
    {-1, -1, -1}, {-1, -1, +1}, {+1, -1, +1}, {+1, -1, -1},
    {-1, +1, -1}, {-1, +1, +1}, {+1, +1, +1}, {+1, +1, -1}
  };

  // Normals for each vertex
  float n[8][3]=
  {
    {-0.577350, -0.577350, -0.577350},
    {-0.577350, -0.577350, 0.577350},
    {0.577350, -0.577350, 0.577350},
    {0.577350, -0.577350, -0.577350},
    {-0.577350, 0.577350, -0.577350},
    {-0.577350, 0.577350, 0.577350},
    {0.577350, 0.577350, 0.577350},
    {0.577350, 0.577350, -0.577350}
  };

  // Texture coords
  float t[4][2] =
  {
    {uvCoords.x, 0}, {0, 0}, {0,uvCoords.y}, {uvCoords.x, uvCoords.y}
  };

  // Vertices
  int faces[6][4] =
  {
    {2, 1, 0, 3}, {5, 6, 7, 4},
    {2, 6, 5, 1}, {1, 5, 4, 0},
    {0, 4, 7, 3}, {6, 2, 3, 7}
  };

  // Indices
  int ind[36] =
  {
    0, 1, 2,
    2, 3, 0,
    4, 5, 7,
    7, 5, 6,
    11,8,9,
    9,10,11,
    12, 13, 15,
    15, 13, 14,
    16, 17, 18,
    18, 19, 16,
    21,22,23,
    23,20,21,
  };

  // Compute the vertices
  for (i = 0; i < 8; i++)
  {
    v[i][0] *= sides.x * 0.5;
    v[i][1] *= sides.y * 0.5;
    v[i][2] *= sides.z * 0.5;
  }

  // For each face
  for (i = 0; i < 6; i++)
  {
    // For each vertex in the face
    for (k=0; k<4; k++)
    {
      subMesh->AddVertex(v[faces[i][k]][0], v[faces[i][k]][1], 
          v[faces[i][k]][2]);
      subMesh->AddNormal(n[faces[i][k]][0], n[faces[i][k]][1], 
          n[faces[i][k]][2]);
      subMesh->AddTexCoord(t[k][0], t[k][1]);
    }
  }

  // Set the indices
  for (i=0;i<36; i++)
    subMesh->AddIndex(ind[i]);

  subMesh->RecalculateNormals();
}

////////////////////////////////////////////////////////////////////////////////
/// Create a Camera mesh
void MeshManager::CreateCamera(const std::string &name, float scale)
{
  int i,k;

  if (this->HasMesh(name))
  {
    return;
  }

  Mesh *mesh = new Mesh();
  mesh->SetName(name);
  this->meshes.insert( std::make_pair(name, mesh) );

  SubMesh *subMesh = new SubMesh();
  mesh->AddSubMesh(subMesh);

  // Vertex values
  float v[8][3] =
  {
    {-1, -1, -1}, {-1, -1, +1}, {+1, -1, +1}, {+1, -1, -1},
    {-1, +1, -1}, {-1, +1, +1}, {+1, +1, +1}, {+1, +1, -1}
  };

  // Normals for each vertex
  float n[8][3]=
  {
    {-0.577350, -0.577350, -0.577350},
    {-0.577350, -0.577350, 0.577350},
    {0.577350, -0.577350, 0.577350},
    {0.577350, -0.577350, -0.577350},
    {-0.577350, 0.577350, -0.577350},
    {-0.577350, 0.577350, 0.577350},
    {0.577350, 0.577350, 0.577350},
    {0.577350, 0.577350, -0.577350}
  };

  // Texture coords
  /*float t[4][2] =
  {
    {uvCoords.x, 0}, {0, 0}, {0,uvCoords.y}, {uvCoords.x, uvCoords.y}
  };*/

  // Vertices
  int faces[6][4] =
  {
    {2, 1, 0, 3}, {5, 6, 7, 4},
    {2, 6, 5, 1}, {1, 5, 4, 0},
    {0, 4, 7, 3}, {6, 2, 3, 7}
  };

  // Indices
  int ind[36] =
  {
    0, 1, 2,
    2, 3, 0,
    4, 5, 7,
    7, 5, 6,
    11,8,9,
    9,10,11,
    12, 13, 15,
    15, 13, 14,
    16, 17, 18,
    18, 19, 16,
    21,22,23,
    23,20,21,
  };

  // Compute the vertices
  for (i = 0; i < 8; i++)
  {
    v[i][0] *= scale * 0.5;
    v[i][1] *= scale * 0.5;
    v[i][2] *= scale * 0.5;
  }

  // For each face
  for (i = 0; i < 6; i++)
  {
    // For each vertex in the face
    for (k=0; k<4; k++)
    {
      subMesh->AddVertex(v[faces[i][k]][0], v[faces[i][k]][1], 
          v[faces[i][k]][2]);
      subMesh->AddNormal(n[faces[i][k]][0], n[faces[i][k]][1], 
          n[faces[i][k]][2]);
      //subMesh->AddTexCoord(t[k][0], t[k][1]);
    }
  }

  // Set the indices
  for (i=0;i<36; i++)
    subMesh->AddIndex(ind[i]);

  mesh->RecalculateNormals();
}

////////////////////////////////////////////////////////////////////////////////
/// Create a cylinder mesh
void MeshManager::CreateCylinder(const std::string &name, float radius, 
                                 float height, int rings, int segments)
{
  Vector3 vert, norm;
  unsigned short verticeIndex = 0;
  unsigned int i,j;
  int ring, seg;
  float deltaSegAngle = (2.0 * M_PI / segments);

  if (this->HasMesh(name))
  {
    return;
  }

  Mesh *mesh = new Mesh();
  mesh->SetName(name);
  this->meshes.insert( std::make_pair(name, mesh) );

  SubMesh *subMesh = new SubMesh();
  mesh->AddSubMesh(subMesh);


  // Generate the group of rings for the sphere
  for (ring = 0; ring <= rings; ring++)
  {
    vert.z = ring * height/rings - height/2.0;

    // Generate the group of segments for the current ring
    for (seg = 0; seg <= segments; seg++)
    {
      vert.y = radius * cosf(seg * deltaSegAngle);
      vert.x = radius * sinf(seg * deltaSegAngle);

      // TODO: Don't think these normals are correct.
      norm = vert;
      norm.Normalize();

      // Add one vertex to the strip which makes up the sphere
      subMesh->AddVertex(vert);
      subMesh->AddNormal(norm);
      subMesh->AddTexCoord((float) seg / (float) segments,
                           (float) ring / (float) rings );

      if (ring != rings)
      {
        // each vertex (except the last) has six indices pointing to it
        subMesh->AddIndex(verticeIndex + segments + 1);
        subMesh->AddIndex(verticeIndex);
        subMesh->AddIndex(verticeIndex + segments);
        subMesh->AddIndex(verticeIndex + segments + 1);
        subMesh->AddIndex(verticeIndex + 1);
        subMesh->AddIndex(verticeIndex);
        verticeIndex++;
      }
    }
  }

  /// The top cap vertex
  subMesh->AddVertex(0,0,height/2.0);
  subMesh->AddNormal(0,0,1);
  subMesh->AddTexCoord(0,0);

  // The bottom cap vertex
  subMesh->AddVertex(0,0,-height/2.0);
  subMesh->AddNormal(0,0,-1);
  subMesh->AddTexCoord(0,0);

  // Create the top fan
  verticeIndex += segments + 1;
  for (seg=0; seg < segments; seg++)
  {
    subMesh->AddIndex(verticeIndex);
    subMesh->AddIndex(verticeIndex - segments + seg);
    subMesh->AddIndex(verticeIndex - segments + seg - 1);
  }

  // Create the bottom fan
  verticeIndex++;
  for (seg=0; seg < segments; seg++)
  {
    subMesh->AddIndex(verticeIndex);
    subMesh->AddIndex(seg);
    subMesh->AddIndex(seg+1);
  }

  // Fix all the normals
  for (i=0; i+3 < subMesh->GetIndexCount(); i+=3)
  {
    norm.Set();

    for (j=0; j<3; j++)
      norm += subMesh->GetNormal( subMesh->GetIndex(i+j) );

    norm /= 3;
    norm.Normalize();

    for (j=0; j<3; j++)
      subMesh->SetNormal(subMesh->GetIndex(i+j), norm );
  }

  mesh->RecalculateNormals();
}

////////////////////////////////////////////////////////////////////////////////
/// Create a cone mesh
void MeshManager::CreateCone(const std::string &name, float radius, 
                             float height, int rings, int segments)
{
  Vector3 vert, norm;
  unsigned short verticeIndex = 0;
  unsigned int i,j;
  int ring, seg;

  if (this->HasMesh(name))
  {
    return;
  }

  Mesh *mesh = new Mesh();
  mesh->SetName(name);
  this->meshes.insert( std::make_pair(name, mesh) );

  SubMesh *subMesh = new SubMesh();
  mesh->AddSubMesh(subMesh);

  if (segments <3)
    segments = 3;

  float deltaSegAngle = (2.0 * M_PI / segments);

  double ringRadius;

  // Generate the group of rings for the cone
  for (ring = 0; ring < rings; ring++)
  {
    vert.z = ring * height/rings - height/2.0;

    ringRadius = ((height - (vert.z+height/2.0)) / height) * radius;

    // Generate the group of segments for the current ring
    for (seg = 0; seg <= segments; seg++)
    {
      vert.y = ringRadius * cosf(seg * deltaSegAngle);
      vert.x = ringRadius * sinf(seg * deltaSegAngle);

      // TODO: Don't think these normals are correct.
      norm = vert;
      norm.Normalize();

      // Add one vertex to the strip which makes up the sphere
      subMesh->AddVertex(vert);
      subMesh->AddNormal(norm);
      subMesh->AddTexCoord((float) seg / (float) segments,
                           (float) ring / (float) rings);

      if (ring != (rings-1))
      {
        // each vertex (except the last) has six indices pointing to it
        subMesh->AddIndex(verticeIndex + segments + 1);
        subMesh->AddIndex(verticeIndex);
        subMesh->AddIndex(verticeIndex + segments);
        subMesh->AddIndex(verticeIndex + segments + 1);
        subMesh->AddIndex(verticeIndex + 1);
        subMesh->AddIndex(verticeIndex);
        verticeIndex++;
      }
    }
  }

  /// The top point vertex
  subMesh->AddVertex(0,0,height/2.0);
  subMesh->AddNormal(0,0,1);
  subMesh->AddTexCoord(0,0);

  // The bottom cap vertex
  subMesh->AddVertex(0,0,-height/2.0);
  subMesh->AddNormal(0,0,-1);
  subMesh->AddTexCoord(0,0);

  // Create the top fan
  verticeIndex += segments+1;
  for (seg=0; seg < segments; seg++)
  {
    subMesh->AddIndex(verticeIndex);
    subMesh->AddIndex(verticeIndex - segments + seg);
    subMesh->AddIndex(verticeIndex - segments + seg - 1);
  }

  // Create the bottom fan
  verticeIndex++;
  for (seg=0; seg < segments; seg++)
  {
    subMesh->AddIndex(verticeIndex);
    subMesh->AddIndex(seg);
    subMesh->AddIndex(seg+1);
  }

  // Fix all the normals
  for (i=0; i+3<subMesh->GetIndexCount(); i+=3)
  {
    norm.Set();

    for (j=0; j<3; j++)
      norm += subMesh->GetNormal( subMesh->GetIndex(i+j) );

    norm /= 3;
    norm.Normalize();

    for (j=0; j<3; j++)
      subMesh->SetNormal(subMesh->GetIndex(i+j), norm );
  }

  mesh->RecalculateNormals();
}

////////////////////////////////////////////////////////////////////////////////
/// Create a tube mesh
void MeshManager::CreateTube(const std::string &name, float innerRadius, 
                             float outterRadius, float height, int rings, 
                             int segments)
{
  Vector3 vert, norm;
  unsigned short verticeIndex = 0;
  int ring, seg;
  float deltaSegAngle = (2.0 * M_PI / segments);

  // Needs at lest 2 rings, and 3 segments
  rings = std::max(rings,1);
  segments = std::max(segments,3);

  float radius= 0;

  radius = outterRadius;

  if (this->HasMesh(name))
    return;

  Mesh *mesh = new Mesh();
  mesh->SetName(name);
  this->meshes.insert( std::make_pair(name, mesh) );
  SubMesh *subMesh = new SubMesh();
  mesh->AddSubMesh(subMesh);

  // Generate the group of rings for the outsides of the cylinder
  for (ring = 0; ring <= rings; ring++)
  {
    vert.z = ring * height/rings - height/2.0;

    // Generate the group of segments for the current ring
    for (seg = 0; seg <= segments; seg++)
    {
      vert.y = radius * cosf(seg * deltaSegAngle);
      vert.x = radius * sinf(seg * deltaSegAngle);

      // TODO: Don't think these normals are correct.
      norm = vert;
      norm.Normalize();

      // Add one vertex to the strip which makes up the tube
      subMesh->AddVertex(vert);
      subMesh->AddNormal(norm);
      subMesh->AddTexCoord((float) seg / (float) segments,
                           (float) ring / (float) rings );

      if (ring != rings)
      {
        // each vertex (except the last) has six indices
        subMesh->AddIndex(verticeIndex + segments + 1);
        subMesh->AddIndex(verticeIndex);
        subMesh->AddIndex(verticeIndex + segments);
        subMesh->AddIndex(verticeIndex + segments + 1);
        subMesh->AddIndex(verticeIndex + 1);
        subMesh->AddIndex(verticeIndex);
      }
      else
      {
        // This indices form the top cap
        subMesh->AddIndex(verticeIndex);
        subMesh->AddIndex(verticeIndex + segments + 1);
        subMesh->AddIndex(verticeIndex+1);
        subMesh->AddIndex(verticeIndex+1);
        subMesh->AddIndex(verticeIndex + segments + 1);
        subMesh->AddIndex(verticeIndex + segments + 2);
      }

      // There indices form the bottom cap
      if (ring == 0 && seg < segments)
      {
        subMesh->AddIndex(verticeIndex+1);
        subMesh->AddIndex(verticeIndex + (segments+1) * (((rings+1)*2)-1));
        subMesh->AddIndex(verticeIndex);
        subMesh->AddIndex(verticeIndex + (segments+1) * (((rings+1)*2)-1) + 1);
        subMesh->AddIndex(verticeIndex + (segments+1) * (((rings+1)*2)-1));
        subMesh->AddIndex(verticeIndex+1);
      }

      verticeIndex++;
    }
  }

  // Generate the group of rings for the inside of the cylinder
  radius = innerRadius;
  for (ring = 0; ring <= rings; ring++)
  {
    vert.z = (height/2.0) - (ring * height/rings);

    // Generate the group of segments for the current ring
    for (seg = 0; seg <= segments; seg++)
    {
      vert.y = radius * cosf(seg * deltaSegAngle);
      vert.x = radius * sinf(seg * deltaSegAngle);

      // TODO: Don't think these normals are correct.
      norm = vert;
      norm.Normalize();

      // Add one vertex to the strip which makes up the tube
      subMesh->AddVertex(vert);
      subMesh->AddNormal(norm);
      subMesh->AddTexCoord((float) seg / (float) segments,
                           (float) ring / (float) rings);

      if (ring != rings)
      {
        // each vertex (except the last) has six indices
        subMesh->AddIndex(verticeIndex + segments + 1);
        subMesh->AddIndex(verticeIndex);
        subMesh->AddIndex(verticeIndex + segments);

        subMesh->AddIndex(verticeIndex + segments + 1);
        subMesh->AddIndex(verticeIndex + 1);
        subMesh->AddIndex(verticeIndex);
      }
      verticeIndex++;
    }
  }

  mesh->RecalculateNormals();
}

