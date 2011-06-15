#include <string.h>
#include <ctype.h>
#include <stdio.h>

#include "Mesh.hh"
#include "STLLoader.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
STLLoader::STLLoader()
  : MeshLoader()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
STLLoader::~STLLoader()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Load a mesh
Mesh *STLLoader::Load( const std::string &filename )
{

  Mesh *mesh = new Mesh();

  FILE *file = fopen(filename.c_str(), "r");

  /*
  std::string extension;
  extension = filename.substr(filename.rfind(".")+1, filename.size());
  std::transform(extension.begin(),extension.end(),extension.begin(),::tolower);
  if (extension == "stl" || extension == "stla")
    this->ReadAscii(file, mesh);
  else if (extension == "stlb")
    this->ReadBinary(file, mesh);
  */

  this->ReadBinary(file, mesh);

  fclose(file);

  return mesh;
}

////////////////////////////////////////////////////////////////////////////////
/// Reads an ASCII STL (stereolithography) file.
void STLLoader::ReadAscii( FILE *filein, Mesh *mesh )
{
  int   count;
  int   ivert;
  char *next;
  float r1;
  float r2;
  float r3;
  float r4;
  char  token[LINE_MAX_LEN];
  int   width;
  char input[LINE_MAX_LEN];

  SubMesh *subMesh = new SubMesh();
  mesh->AddSubMesh(subMesh);

  // Read the next line of the file into INPUT. 
  while ( fgets ( input, LINE_MAX_LEN, filein ) != NULL ) 
  {

    // Advance to the first nonspace character in INPUT. 
    for ( next = input; *next != '\0' && isspace(*next); next++ );

    // Skip blank lines and comments. 
    if ( *next == '\0' || *next == '#' || *next == '!' || *next == '$' )
      continue;

    //Extract the first word in this line. 
    sscanf ( next, "%s%n", token, &width );

    //Set NEXT to point to just after this token. 
    next = next + width;

    //FACET
    if (this->Leqi(token, (char*)"facet"))
    { 
      Vector3 normal;

      //Get the XYZ coordinates of the normal vector to the face. 
      sscanf ( next, "%*s %e %e %e", &r1, &r2, &r3 );  

      normal.x = r1;
      normal.y = r2;
      normal.z = r3;

      if (fgets ( input, LINE_MAX_LEN, filein ) == NULL)
        std::cerr << "Error..\n";

      ivert = 0;

      for ( ;; ) 
      {
        Vector3 vertex;
        if (fgets ( input, LINE_MAX_LEN, filein ) == NULL)
          std::cerr << "Error...\n";

        count = sscanf ( input, "%*s %e %e %e", &r1, &r2, &r3 );

        if ( count != 3 ) 
          break;

        vertex.x = r1;
        vertex.y = r2;
        vertex.z = r3;

        //if (!subMesh->HasVertex(vertex))  // skipping vertices breaks trimesh
        //{
          subMesh->AddVertex(vertex);
          subMesh->AddNormal(normal);
        //}
        subMesh->AddIndex( subMesh->GetVertexIndex(vertex) );

        ivert = ivert + 1;
      } 

      if (fgets ( input, LINE_MAX_LEN, filein ) == NULL)
        printf("Error...\n");
    }
    // COLOR 
    else if ( this->Leqi ( token, (char*)"color" )) 
    {
      sscanf ( next, "%*s %f %f %f %f", &r1, &r2, &r3, &r4 );
    }
    // SOLID 
    else if ( this->Leqi ( token, (char*)"solid" ))
    {
    }
    // ENDSOLID 
    else if ( this->Leqi ( token, (char*)"endsolid" ))
    {
    }
    // Unexpected or unrecognized. 
    else 
    {
      printf ( "\n" );
      printf ( "stl - Fatal error!\n" );
      printf ( "  Unrecognized first word on line.\n" );
    }
  }

}

////////////////////////////////////////////////////////////////////////////////
/// Reads a binary STL (stereolithography) file.
void STLLoader::ReadBinary ( FILE *filein, Mesh *mesh )
{
  int attribute = 0;
  char c;
  int i;
  int iface;

  SubMesh *subMesh = new SubMesh();
  mesh->AddSubMesh(subMesh);

  //80 byte Header.
  for ( i = 0; i < 80; i++ ) 
    c = (char)fgetc(filein);

  //Number of faces.
  int face_num = this->LongIntRead(filein);

  //For each (triangular) face,
  //  components of normal vector,
  //  coordinates of three vertices,
  //  2 byte "attribute".
  for (iface = 0; iface < face_num; iface++)
  {
    Vector3 normal;

    // Get the normal for the face
    normal.x =  this->FloatRead(filein);
    normal.y =  this->FloatRead(filein);
    normal.z =  this->FloatRead(filein);

    // Get three vertices
    for (i = 0; i < 3; i++) 
    {
      Vector3 vertex;

      vertex.x = this->FloatRead(filein);
      vertex.y = this->FloatRead(filein);
      vertex.z = this->FloatRead(filein);

      //if (!subMesh->HasVertex(vertex))  // skipping vertices breaks trimesh
      //{
        subMesh->AddVertex(vertex);
        subMesh->AddNormal(normal);
      //}
      subMesh->AddIndex( subMesh->GetVertexIndex(vertex) );
    }

    attribute = ShortIntRead ( filein );
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Write the STL binary file out
/// this subroutine assumes STL mesh corresponds to the one and only SubMesh
/// in Mesh*,
void STLLoader::WriteBinary (FILE *fileout, const Mesh* mesh, Pose3d pose)
{
  int submesh_count = mesh->GetSubMeshCount();
  if (submesh_count > 1) std::cout << "WARNING: we can only handle one submesh at this time\n";
  if (submesh_count == 0)
  {
    std::cout << "WARNING: no submeshes, return without writing STLB\n";
    return;
  }

  const SubMesh* subMesh = mesh->GetSubMesh(0);

  int bytes = 0;

  // 80 byte header, use empty string
  for (int i = 0; i < 80; i++ )
    bytes = bytes + CharWrite ( fileout, ' ' );

  // number of faces is number of vertices / 3
  int vertex_num = subMesh->GetVertexCount();

  // write number of faces
  bytes = bytes + LongIntWrite ( fileout, vertex_num/3 );

  // write per face (1 normal repeated 3 times, 3 corners)
  for (int ivertex = 0; ivertex < vertex_num; ivertex += 3 )
  {
    // write face normal
    Vector3 normal = subMesh->GetNormal(ivertex);
    normal = pose.rot * normal; // rotate surface normals
    bytes = bytes + FloatWrite ( fileout, normal.x );
    bytes = bytes + FloatWrite ( fileout, normal.y );
    bytes = bytes + FloatWrite ( fileout, normal.z );

    // get vertices
    Vector3 vertex1 = subMesh->GetVertex(ivertex);
    Vector3 vertex2 = subMesh->GetVertex(ivertex+1);
    Vector3 vertex3 = subMesh->GetVertex(ivertex+2);

    // rotate and translate points
    vertex1 = pose.pos + pose.rot * vertex1;
    vertex2 = pose.pos + pose.rot * vertex2;
    vertex3 = pose.pos + pose.rot * vertex3;

    // write coord 1
    bytes = bytes + FloatWrite ( fileout, vertex1.x );
    bytes = bytes + FloatWrite ( fileout, vertex1.y );
    bytes = bytes + FloatWrite ( fileout, vertex1.z );

    // write coord 2
    bytes = bytes + FloatWrite ( fileout, vertex2.x );
    bytes = bytes + FloatWrite ( fileout, vertex2.y );
    bytes = bytes + FloatWrite ( fileout, vertex2.z );

    // write coord 3
    bytes = bytes + FloatWrite ( fileout, vertex3.x );
    bytes = bytes + FloatWrite ( fileout, vertex3.y );
    bytes = bytes + FloatWrite ( fileout, vertex3.z );

    int attribute = 0; // defined by binary stl format
    bytes = bytes + ShortIntWrite ( fileout, attribute );
  }
  std::cout << "binary STL file has [" << bytes << "] bytes.\n";


}

////////////////////////////////////////////////////////////////////////////////
/// Compares two strings for equality, disregarding case.
bool STLLoader::Leqi ( char* string1, char* string2 )
{
  int i;
  int nchar;
  int nchar1;
  int nchar2;

  nchar1 = strlen ( string1 );
  nchar2 = strlen ( string2 );

  if ( nchar1 < nchar2 ) 
    nchar = nchar1;
  else
    nchar = nchar2;

  // The strings are not equal if they differ over their common length.
  for ( i = 0; i < nchar; i++ ) 
    if ( toupper ( string1[i] ) != toupper ( string2[i] ) )
      return false;

  //The strings are not equal if the longer one includes nonblanks in the tail.
  if ( nchar1 > nchar ) 
  {
    for ( i = nchar; i < nchar1; i++ ) 
      if ( string1[i] != ' ' ) 
        return false;
  }
  else if ( nchar2 > nchar ) 
  {
    for ( i = nchar; i < nchar2; i++ ) 
      if ( string2[i] != ' ' ) 
        return false;
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////
/// Finds if a vector occurs in a table.
int STLLoader::RcolFind ( float a[][COR3_MAX], int m, int n, float r[] ) 
{
  int i;
  int icol;
  int j;

  icol = -1;

  for ( j = 0; j < n; j++ ) 
  {
    for ( i = 0; i < m; i++ ) 
    {
      if ( a[i][j] != r[i] ) 
        break;
      if ( i == m-1 )
        return j;
    }
  }

  return icol;
}

////////////////////////////////////////////////////////////////////////////////
/// Read 1 float from a binary file.
float STLLoader::FloatRead ( FILE *filein )
{
  float rval;
  if (fread ( &rval, sizeof ( float ), 1, filein ) == 0)
    printf("Error...\n");

  return rval;
}

////////////////////////////////////////////////////////////////////////////////
/// Reads a long int from a binary file.
uint32_t STLLoader::LongIntRead ( FILE *filein )
{
  union 
  {
    uint32_t yint;
    char ychar[4];
  } y;

  y.ychar[0] = fgetc ( filein );
  y.ychar[1] = fgetc ( filein );
  y.ychar[2] = fgetc ( filein );
  y.ychar[3] = fgetc ( filein );

  return y.yint;
}

////////////////////////////////////////////////////////////////////////////////
/// Reads a short int from a binary file.
uint16_t STLLoader::ShortIntRead ( FILE *filein )
{
  uint8_t  c1;
  uint8_t  c2;
  uint16_t      ival;

  c1 = fgetc ( filein );
  c2 = fgetc ( filein );

  ival = c1 | ( c2 << 8 );

  return ival;
}

uint16_t STLLoader::CharWrite ( FILE *fileout, char c )
{
  fputc ( c, fileout );
  return 1;
}

uint16_t STLLoader::LongIntWrite ( FILE *fileout, long int int_val )
{
  union
  {
    long int yint;
    char ychar[4];
  } y;

  y.yint = int_val;

  fputc ( y.ychar[0], fileout );
  fputc ( y.ychar[1], fileout );
  fputc ( y.ychar[2], fileout );
  fputc ( y.ychar[3], fileout );

  return 4;
}
uint16_t STLLoader::FloatWrite ( FILE *fileout, float float_val )
{
  int nbytes = sizeof ( float );
  fwrite ( &float_val, nbytes, 1, fileout );
  return nbytes;
}

uint16_t STLLoader::ShortIntWrite ( FILE *fileout, short int short_int_val )
{
  union
  {
    short int yint;
    char ychar[2];
  } y;

  y.yint = short_int_val;

  fputc ( y.ychar[0], fileout );
  fputc ( y.ychar[1], fileout );

  return 2;
}
