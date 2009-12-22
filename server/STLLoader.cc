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
  std::string extension;
 
  extension = filename.substr(filename.rfind(".")+1, filename.size());

  Mesh *mesh = new Mesh();

  FILE *file = fopen(filename.c_str(), "r");

  /*if (extension == "stl" || extension == "stla")
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

        if (!subMesh->HasVertex(vertex))
        {
          subMesh->AddVertex(vertex);
          subMesh->AddNormal(normal);
        }
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
  short int attribute = 0;
  char c;
  int i;
  int iface;
  int face_num;

  SubMesh *subMesh = new SubMesh();
  mesh->AddSubMesh(subMesh);

  //80 byte Header.
  for ( i = 0; i < 80; i++ ) 
    c = (char)fgetc(filein);

  //Number of faces.
  face_num = this->LongIntRead(filein);

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

      if (!subMesh->HasVertex(vertex))
      {
        subMesh->AddVertex(vertex);
        subMesh->AddNormal(normal);
      }
      subMesh->AddIndex( subMesh->GetVertexIndex(vertex) );
    }

    attribute = ShortIntRead ( filein );
  }
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
