#ifndef STLLOADER_HH
#define STLLOADER_HH

#include <stdint.h>

#include "MeshLoader.hh"
#include "Pose3d.hh"

#define LINE_MAX_LEN 256
#define COR3_MAX 200000
#define ORDER_MAX 10
#define FACE_MAX 200000


namespace gazebo
{
  /// \brief Class used to load STL mesh files
  class STLLoader : public MeshLoader
  {
    /// \brief Constructor
    public: STLLoader();

    /// \brief Destructor
    public: virtual ~STLLoader();

    /// \brief Load a mesh
    public: virtual Mesh *Load( const std::string &filename );

    /// Reads an ASCII STL (stereolithography) file.
    private: void ReadAscii( FILE *filein, Mesh *mesh );

    /// Reads a binary STL (stereolithography) file.
    private: void ReadBinary ( FILE *filein, Mesh *mesh );

    /// Compares two strings for equality, disregarding case.
    private: bool Leqi ( char* string1, char* string2 );

    /// Finds if a vector occurs in a table.
    private: int RcolFind ( float a[][COR3_MAX], int m, int n, float r[] );

    /// Reads a long int from a binary file.
    private: uint32_t LongIntRead ( FILE *filein );

    /// Reads a short int from a binary file.
    private: uint16_t ShortIntRead ( FILE *filein );

    /// Read 1 float from a binary file.
    private: float FloatRead ( FILE *filein );

    /// Write STL Binary file
    public: void WriteBinary (FILE *fileout, const Mesh* mesh, Pose3d pose);

    /// helper function to write STL mesh
    private: uint16_t CharWrite ( FILE *fileout, char c );
    private: uint16_t LongIntWrite ( FILE *fileout, long int int_val );
    private: uint16_t FloatWrite ( FILE *fileout, float float_val );
    private: uint16_t ShortIntWrite ( FILE *fileout, short int short_int_val );


  };
}

#endif
