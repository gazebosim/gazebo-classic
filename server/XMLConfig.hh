#ifndef XMLCONFIG_HH
#define XMLCONFIG_HH

#include <libxml/parser.h>
#include <string>

#include "Vector3.hh"
#include "Quatern.hh"
#include "Time.hh"

// Forward declarations
class XMLConfigNode;

class XMLConfig
{
  // Constructor
  public: XMLConfig();

  // Destructor
  public: ~XMLConfig();

  // Load config from an XML file
  public: int Load( std::string filename );

  // Load config from an XML string
  public: int LoadString( std::string str );

  // Save config back into file
  // Set filename to NULL to save back into the original file
  public: int Save( std::string filename );

  // Get the root node
  public: XMLConfigNode *GetRootNode() const;

  // Create wrappers
  private: XMLConfigNode *CreateNodes( XMLConfigNode *parent, 
                                       xmlNodePtr xmlNode );
  // File name
  public: std::string filename;
  
  // XML data
  private: xmlDocPtr xmlDoc;

  // The root of the tree
  private: XMLConfigNode *root;
};


// Class encapsulating a single xml node
class XMLConfigNode
{
  // Constructor
  public: XMLConfigNode( XMLConfig *cfg, XMLConfigNode *parent, 
                        xmlNodePtr xmlNode, xmlDocPtr xmlDoc );

  // Destructors
  public: ~XMLConfigNode();

  // Get the node name
  public: std::string GetName();

  // Get the name space prefix
  public: std::string GetNSPrefix();

  // Get the next sibling of this node
  public: XMLConfigNode *GetNext();
  public: XMLConfigNode *GetNext(std::string name);
  public: XMLConfigNode *GetNextByNSPrefix(std::string name);

  // Get the first child of this node
  public: XMLConfigNode *GetChild();

  // Get a child based on a name. Returns null if not found
  public: XMLConfigNode *GetChild( std::string name );

  // Get the first child with the specified namespace prefix
  public: XMLConfigNode *GetChildByNSPrefix( std::string prefix );

  // Move child pointer back to beginning
  public: XMLConfigNode *Rewind();

  // Print (for debugging purposes)
  public: void Print();

  // Return the value of the current node
  public: std::string GetValue();

  // Get an attribute string value
  public: std::string GetString( std::string key, std::string def, 
                                 int require = 0 );

  // Get a attribute character value
  public: unsigned char GetChar( std::string key, char def, int require = 0 );

  // Get a file name.  Always returns an absolute path.  If the filename
  // is entered as a relative path, we prepend the world file path.
  public: std::string GetFilename( std::string key, std::string def, 
                                   int require = 0);

  // Get an integer
  public: int GetInt( std::string key, int def, int require = 0 );

  // Get a double
  public: double GetDouble( std::string key, double def, int require = 0 );

  // Get a float
  public: float GetFloat( std::string key, float def, int require = 0 );

  // Get a boolean
  public: bool GetBool( std::string key, bool def, int require = 0 );

  // Get an attribute length value (return value in meters)
  public: double GetLength( std::string key, double def, int require = 0 );

  // Get an attribute time value (return value in seconds)
  public: gazebo::Time GetTime( std::string key, double def, int require = 0 );

  // Get a position
  public: Vector3 GetVector3( std::string key, Vector3 def );

  // Get a rotation
  public: Quatern GetRotation( std::string key, Quatern def );

  // Get an attribute tuple value
  public: std::string GetTupleString( std::string key, int index, 
                                      std::string def );

  // Get an attribute tuple int value
  public: int GetTupleInt( std::string key, int index, int def );

  // Get an attribute tuple double value
  public: double GetTupleDouble( std::string key, int index, double def );

  // Get an attribute tuple length value (return value in meters)
  public: double GetTupleLength( std::string key, int index, double def );

  // Get an attribute tuple angle value (return value in radians)
  public: double GetTupleAngle( std::string key, int index, double def );

  // Get a node's value, which is either a attribute or child node value.
  protected: xmlChar* GetNodeValue( std::string key );

  // Our document
  private: XMLConfig *config;
  
  // Our parent
  private: XMLConfigNode *parent;
  
  // Our siblings
  private: XMLConfigNode *next, *prev;
  
  // Our children
  private: XMLConfigNode *childFirst, *childLast;

  // XML data
  private: xmlNodePtr xmlNode;

  // XML data
  private: xmlDocPtr xmlDoc;
};

#endif

