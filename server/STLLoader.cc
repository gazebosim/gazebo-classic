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

  if (extension == "stl" || extension == "stla")
    this->ReadAscii(file, mesh);
  else if (extension == "stlb")
    this->ReadBinary(file, mesh);

  return mesh;
}

////////////////////////////////////////////////////////////////////////////////
/// Reads an ASCII STL (stereolithography) file.
void STLLoader::ReadAscii( FILE *filein, Mesh *mesh )
{
  int   count;
  int   i;
  int   icor3;
  int   ivert;
  char *next;
  float r1;
  float r2;
  float r3;
  float r4;
  float temp[3];
  char  token[LINE_MAX_LEN];
  int   width;
  float face_normal[3][FACE_MAX];
  int face[ORDER_MAX][FACE_MAX];
  int face_order[FACE_MAX];
  int    vertex_material[ORDER_MAX][FACE_MAX];
  float  vertex_normal[3][ORDER_MAX][FACE_MAX];
  int text_num;
  int face_num;
  int cor3_num;
  int object_num;
  int dup_num;
  char input[LINE_MAX_LEN];
  float cor3[3][COR3_MAX];

  // Read the next line of the file into INPUT. 
  while ( fgets ( input, LINE_MAX_LEN, filein ) != NULL ) 
  {
    text_num = text_num + 1;

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
      //Get the XYZ coordinates of the normal vector to the face. 
      sscanf ( next, "%*s %e %e %e", &r1, &r2, &r3 );  

      if ( face_num < FACE_MAX ) 
      {
        face_normal[0][face_num] = r1;
        face_normal[1][face_num] = r2;
        face_normal[2][face_num] = r3;
      }

      fgets ( input, LINE_MAX_LEN, filein );
      text_num = text_num + 1;

      ivert = 0;

      for ( ;; ) 
      {
        fgets ( input, LINE_MAX_LEN, filein );
        text_num = text_num + 1;

        count = sscanf ( input, "%*s %e %e %e", &r1, &r2, &r3 );

        if ( count != 3 ) 
          break;

        temp[0] = r1;
        temp[1] = r2;
        temp[2] = r3;

        if ( cor3_num < 1000 )
          icor3 = this->RcolFind(cor3, 3, cor3_num, temp);
        else
          icor3 = -1;

        if ( icor3 == -1 ) 
        {
          icor3 = cor3_num;

          if ( cor3_num < COR3_MAX ) 
          {
            for ( i = 0; i < 3; i++ )
              cor3[i][cor3_num] = temp[i];
          }
          cor3_num = cor3_num + 1;
        }
        else
          dup_num = dup_num + 1;

        if ( ivert < ORDER_MAX && face_num < FACE_MAX ) 
        {
          face[ivert][face_num] = icor3;
          vertex_material[ivert][face_num] = 0;
          for ( i = 0; i < 3; i++ )
            vertex_normal[i][ivert][face_num] = face_normal[i][face_num];
        }

        ivert = ivert + 1;
      } 

      fgets ( input, LINE_MAX_LEN, filein );
      text_num = text_num + 1;

      if ( face_num < FACE_MAX ) 
        face_order[face_num] = ivert;

      face_num = face_num + 1;

    }
    // COLOR 
    else if ( this->Leqi ( token, (char*)"color" )) 
      sscanf ( next, "%*s %f %f %f %f", &r1, &r2, &r3, &r4 );
    // SOLID 
    else if ( this->Leqi ( token, (char*)"solid" ))
      object_num = object_num + 1;
    // ENDSOLID 
    else if ( this->Leqi ( token, (char*)"endsolid" ))
    {
    }
    // Unexpected or unrecognized. 
    else 
    {
      printf ( "\n" );
      printf ( "STLA_READ - Fatal error!\n" );
      printf ( "  Unrecognized first word on line.\n" );
    }
  }

  SubMesh *subMesh = new SubMesh();
  mesh->AddSubMesh(subMesh);

  for (int i=0; i < face_num; i++)
    subMesh->AddVertex( cor3[0][i], cor3[1][i], cor3[2][i] );

  for (int i=0; i < face_num; i++)
    for (int j=0; j < face_order[i]; j++)
      subMesh->AddIndex(face[j][i]);
}

////////////////////////////////////////////////////////////////////////////////
/// Reads a binary STL (stereolithography) file.
void STLLoader::ReadBinary ( FILE *filein, Mesh *mesh )
{
  short int attribute = 0;
  char c;
  float cvec[3];
  int icor3;
  int i;
  int iface;
  int ivert;
  int face_num;
  int face_order[FACE_MAX];
  int face_material[FACE_MAX];
  int bytes_num;
  int cor3_num;
  int dup_num;
  float cor3[3][COR3_MAX];
  float face_normal[3][FACE_MAX];
  int face[3][FACE_MAX];

  //80 byte Header.
  for ( i = 0; i < 80; i++ ) 
  {
    c = (char)fgetc(filein);
    bytes_num = bytes_num + 1;
  }

  //Number of faces.
  face_num = this->LongIntRead(filein);
  bytes_num = bytes_num + 4;

  //For each (triangular) face,
  //  components of normal vector,
  //  coordinates of three vertices,
  //  2 byte "attribute".
  for (iface = 0; iface < face_num; iface++)
  {
    face_order[iface] = 3;
    face_material[iface] = 0;

    for ( i = 0; i < 3; i++ ) 
    {
      face_normal[i][iface] = this->FloatRead(filein);
      bytes_num = bytes_num + 4;
    }

    for (ivert = 0; ivert < face_order[iface]; ivert++) 
    {
      for ( i = 0; i < 3; i++ ) 
      {
        cvec[i] = this->FloatRead(filein);
        bytes_num = bytes_num + 4;
      }

      if ( cor3_num < 1000 ) 
        icor3 = this->RcolFind(cor3, 3, cor3_num, cvec);
      else
        icor3 = -1;

      if (icor3 == -1) 
      {
        icor3 = cor3_num;
        if (cor3_num < COR3_MAX) 
        {
          cor3[0][cor3_num] = cvec[0];
          cor3[1][cor3_num] = cvec[1];
          cor3[2][cor3_num] = cvec[2];
        }
        cor3_num = cor3_num + 1;
      }
      else 
        dup_num = dup_num + 1;

      face[ivert][iface] = icor3;
    }

    attribute = ShortIntRead ( filein );

    bytes_num = bytes_num + 2;
  }

  SubMesh *subMesh = new SubMesh();
  mesh->AddSubMesh(subMesh);

  for (int i=0; i < face_num; i++)
    subMesh->AddVertex( cor3[0][i], cor3[1][i], cor3[2][i] );

  for (int i=0; i < face_num; i++)
    for (int j=0; j < face_order[i]; j++)
      subMesh->AddIndex(face[j][i]);
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
  fread ( &rval, sizeof ( float ), 1, filein );
  return rval;
}

////////////////////////////////////////////////////////////////////////////////
/// Reads a long int from a binary file.
long int STLLoader::LongIntRead ( FILE *filein )
{
  union 
  {
    long int yint;
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
short int STLLoader::ShortIntRead ( FILE *filein )
{
  unsigned char  c1;
  unsigned char  c2;
  short int      ival;

  c1 = fgetc ( filein );
  c2 = fgetc ( filein );

  ival = c1 | ( c2 << 8 );

  return ival;
}
