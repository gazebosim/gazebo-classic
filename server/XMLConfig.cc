/* ==================================================================
 *                            XMLConfig - Source
 * 
 * Description:
 *    See header
 *
 * -------------------------------------------------------------------
 *  Revisions:
 *
 *  CVS: $Id: XMLConfig.cc,v 1.1.2.1 2006/12/16 22:41:14 natepak Exp $
 *
 *  12/22/03  NPK  Creation
 *
 * ==================================================================
*/

#include <assert.h>
#include <string.h>
#include <float.h>
#include <math.h>

#include "XMLConfig.hh"
#include "Global.hh"

////////////////////////////////////////////////////////////////////////////
// Standard constructor
XMLConfig::XMLConfig()
{
  this->root = NULL;

  return;
}


////////////////////////////////////////////////////////////////////////////
// Standard destructor
XMLConfig::~XMLConfig()
{
  if (this->root)
    delete this->root;

  return;
}


////////////////////////////////////////////////////////////////////////////
// Load world from file
int XMLConfig::Load( std::string filename )
{
  this->filename = filename;

  // Enable line numbering
  xmlLineNumbersDefault( 1 );

  // Load the file
  this->xmlDoc = xmlParseFile( this->filename.c_str() );
  if (xmlDoc == NULL)
  {
    std::cerr << "Unable to parse xml file: " << this->filename << std::endl;
    return -1;
  }

  // Create wrappers for all the nodes (recursive)
  this->root = this->CreateNodes( NULL, xmlDocGetRootElement(this->xmlDoc) );
  if (this->root == NULL)
  {
    std::cerr << "Empty document [" << this->filename << std::endl;
    return -1;
  }

  return 0;
}

int XMLConfig::LoadString( std::string str )
{
  // Enable line numbering
  xmlLineNumbersDefault( 1 );

  // Load the file
  this->xmlDoc = xmlParseDoc( (xmlChar*)(str.c_str()) );
  if (xmlDoc == NULL)
  {
    std::cerr << "unable to parse [" << str << "]";
    return -1;
  }

  // Create wrappers for all the nodes (recursive)
  this->root = this->CreateNodes( NULL,
      xmlDocGetRootElement(this->xmlDoc) );

  if (this->root == NULL)
  {
    std::cerr << "Empty document [" << str << "\n";
    return -1;
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////
// Get the root node
XMLConfigNode *XMLConfig::GetRootNode() const
{
  return this->root;
}


////////////////////////////////////////////////////////////////////////////
// Create wrappers
XMLConfigNode *XMLConfig::CreateNodes( XMLConfigNode *parent, 
                                       xmlNodePtr xmlNode )
{
  XMLConfigNode *self = NULL;

  // No need to create wrappers around text and blank nodes
  if( !xmlNodeIsText( xmlNode ) && !xmlIsBlankNode( xmlNode ) )
  {

    // Create a new node
    self = new XMLConfigNode(this, parent, xmlNode,xmlDoc);

    // Create our children
    for ( xmlNode = xmlNode->xmlChildrenNode; xmlNode != NULL; 
         xmlNode = xmlNode->next)
      this->CreateNodes( self, xmlNode );
  }

  return self;
}



////////////////////////////////////////////////////////////////////////////
// Constructor
XMLConfigNode::XMLConfigNode( XMLConfig *cfg, XMLConfigNode *parent, 
                              xmlNodePtr xmlNode, xmlDocPtr xmlDoc )
{
  this->config = cfg;

  this->parent = parent;
  this->prev = NULL;
  this->next = NULL;
  this->childFirst = NULL;
  this->childLast = NULL;

  // Link ourself to our parent
  if (this->parent)
  {
    if (!this->parent->childFirst)
      this->parent->childFirst = this;

    this->prev = this->parent->childLast;
    this->parent->childLast = this;
  }

  // Link our self to our siblings
  if (this->prev)
    this->prev->next = this;

  this->xmlNode = xmlNode;
  this->xmlDoc = xmlDoc;

  return;
}


////////////////////////////////////////////////////////////////////////////
// Destructor
XMLConfigNode::~XMLConfigNode()
{
  // Delete all our children first
  if (this->childFirst)
    delete this->childFirst;
    
  // Unlink ourself from our siblings
  if (this->prev)
    this->prev->next = this->next;

  if (this->next)
    this->next->prev = this->prev;

  // Unlink ourself from our parent
  if (this->parent)
  {
    if (this->parent->childFirst == this)
      this->parent->childFirst = this->next;

    if (this->parent->childLast == this)
      this->parent->childLast = this->prev;
  }

  return;
}


////////////////////////////////////////////////////////////////////////////
// Get the node name
std::string XMLConfigNode::GetName()
{
  return (const char*)(this->xmlNode->name);
}

////////////////////////////////////////////////////////////////////////////
// Get the Name Space Prefix
std::string XMLConfigNode::GetNSPrefix()
{
  if( !this->xmlNode->ns )
    return "";
  else
    return (const char*)this->xmlNode->ns->prefix;
}

////////////////////////////////////////////////////////////////////////////
// Get the next sibling of this node
XMLConfigNode *XMLConfigNode::GetNext()
{
  return this->next;
}

////////////////////////////////////////////////////////////////////////////
// Get the next sibling of this node
XMLConfigNode *XMLConfigNode::GetNext(std::string name)
{
  XMLConfigNode *tmp = NULL;

  for (tmp = this->next; tmp != NULL; tmp = tmp->GetNext() )
  {
    if (tmp->xmlNode->name && name == (const char*)tmp->xmlNode->name)
      break;
  }

  return tmp; 
}

////////////////////////////////////////////////////////////////////////////
// Get the next sibling of this node according the namespace prefix
XMLConfigNode *XMLConfigNode::GetNextByNSPrefix(std::string prefix)
{
  XMLConfigNode *tmp = NULL;

  for (tmp = this->next; tmp != NULL; tmp = tmp->GetNext() )
  {
    if (tmp->xmlNode->ns && prefix == (const char*)tmp->xmlNode->ns->prefix)
      break;
  }

  return tmp; 

}

////////////////////////////////////////////////////////////////////////////
// Get the first child of this node
XMLConfigNode *XMLConfigNode::GetChild()
{
  return this->childFirst;
}

////////////////////////////////////////////////////////////////////////////
// Get the first child with the appropriate NS prefix
XMLConfigNode *XMLConfigNode::GetChildByNSPrefix(std::string prefix )
{
  XMLConfigNode *tmp = NULL;

  for (tmp = this->childFirst; tmp != NULL; tmp = tmp->GetNext() )
  {
    if (tmp->xmlNode->ns && prefix == (const char*)tmp->xmlNode->ns->prefix)
      break;
  }

  return tmp; 

}

////////////////////////////////////////////////////////////////////////////
// Rewind the node pointer to the first siblind
XMLConfigNode *XMLConfigNode::Rewind()
{
  XMLConfigNode *result = this;

  while (result && result->prev)
  {
    result = result->prev;
  }

  return result;
}

////////////////////////////////////////////////////////////////////////////
// Get a child based on a name. Returns null if not found
XMLConfigNode *XMLConfigNode::GetChild( std::string name )
{
  XMLConfigNode *tmp;
  for (tmp = this->childFirst; tmp != NULL && 
      name != (const char*)tmp->xmlNode->name; tmp = tmp->GetNext() );

  return tmp;
}


////////////////////////////////////////////////////////////////////////////
// Print (for debugging purposes)
void XMLConfigNode::Print()
{
  XMLConfigNode *node;

  std::cout << "name = [" << (const char*) this->xmlNode->name 
                      << "]\n";

  std::cout << "id = [" << xmlGetProp(this->xmlNode, (xmlChar*) "id")
                      << "]\n";

  // Recurse
  for (node = this->childFirst; node != NULL; node = node->next)
  {
    if (node != NULL)
      node->Print();
  }

  return;
}


////////////////////////////////////////////////////////////////////////////
// Get a value associated with a node.
xmlChar* XMLConfigNode::GetNodeValue( std::string key )
{
  xmlChar *value=NULL;

  // First check if the key is an attribute
  if (xmlHasProp( this->xmlNode, (xmlChar*) key.c_str() ))
  {
    value = xmlGetProp( this->xmlNode, (xmlChar*) key.c_str() );

    // If not an attribute, then it should be a child node
  }
  else if (key == this->GetName())
  {
    value = xmlNodeListGetString( this->xmlDoc, 
                                  this->xmlNode->xmlChildrenNode, 1);
  }
  else
  {
    XMLConfigNode *currNode;

    currNode = this->childFirst;

    // Loop through children
    while (currNode)
    {
      // If the name matches, then return its value
      if (key == currNode->GetName())
      {
        value = xmlNodeListGetString( this->xmlDoc,
                                      currNode->xmlNode->xmlChildrenNode, 1 );
        break;
      }

      currNode = currNode->next;
    }
  }

  return value;
}


////////////////////////////////////////////////////////////////////////////
// Get the value of this node
std::string XMLConfigNode::GetValue()
{
  return (const char*)xmlNodeListGetString(this->xmlDoc, this->xmlNode->xmlChildrenNode, 1);
}


////////////////////////////////////////////////////////////////////////////
// Get a string value.
std::string XMLConfigNode::GetString( std::string key, std::string def, 
                                      int require)
{
  xmlChar *value = this->GetNodeValue( key );

  if (!value && require)
  {
    return "";
  }
  else if( !value )
    return def;

  // TODO: cache the value somewhere (currently leaks)
  return (char *)value;
}

unsigned char XMLConfigNode::GetChar( std::string key, char def, int require )
{
  unsigned char result = ' ';

  xmlChar *value = this->GetNodeValue( key );

  if (!value && require)
  {
    return result;
  }
  else if( !value )
    return def;

  result = value[0];

  xmlFree( value );

  return result;
}

///////////////////////////////////////////////////////////////////////////
// Get a file name.  Always returns an absolute path.  If the filename
// is entered as a relative path, we prepend the world file path.
std::string XMLConfigNode::GetFilename( std::string key, std::string def, 
                                        int require)
{
  std::string filename = this->GetString( key, def, require );

  if (filename.empty())
    return "";

  if (filename[0] == '/' || filename[0] == '~')
    return filename;
  else 
  {
    std::string result;

    if (this->config->filename[0] != '/' && this->config->filename[0] != '~')
      result = "/";

    unsigned int last = this->config->filename.rfind("/");
    if (last==0 || last+1 != this->config->filename.size())
      result += this->config->filename + "/" + filename;
    else
      result += this->config->filename.substr(0,last) + "/" + filename;

    return result;
  }
}


////////////////////////////////////////////////////////////////////////////
// Get an integer
int XMLConfigNode::GetInt( std::string key, int def, int require )
{
  xmlChar *value = this->GetNodeValue( key );

  if (!value && require)
  {
    return -1;
  }
  else if( !value )
    return def;

  return atoi((const char*) value);
}


////////////////////////////////////////////////////////////////////////////
// Get a double
double XMLConfigNode::GetDouble( std::string key, double def, int require )
{
  xmlChar *value = this->GetNodeValue( key );

  if (!value && require)
  {
    return -1;
  }
  else if( !value )
    return def;

  return atof((const char*) value);
}

////////////////////////////////////////////////////////////////////////////
// Get a float
float XMLConfigNode::GetFloat( std::string key, float def, int require )
{
  xmlChar *value = this->GetNodeValue( key );

  if (!value && require)
  {
    return -1;
  }
  else if( !value )
    return def;

  return (float)(atof((const char*) value));
}


////////////////////////////////////////////////////////////////////////////
// Get a boolean
bool XMLConfigNode::GetBool( std::string key, bool def, int require )
{
  xmlChar *value = this->GetNodeValue( key );

  if (!value && require)
  {
    return -1;
  }
  else if( !value )
    return def;

  if (strcmp((const char*) value, "true") == 0)
    return 1;
  
  return atoi((const char*) value);
}


////////////////////////////////////////////////////////////////////////////
// Get a length
double XMLConfigNode::GetLength( std::string key, double def, int require )
{
  double length = this->GetDouble(key, def, require);

  // Do unit conversion here
  
  return length;
}


////////////////////////////////////////////////////////////////////////////
// Get a time
gazebo::Time XMLConfigNode::GetTime( std::string key, double def, int require )
{
  gazebo::Time time(this->GetDouble(key, def, require));
  return time;
}

////////////////////////////////////////////////////////////////////////////
// Get a position
Vector3 XMLConfigNode::GetVector3( std::string key, Vector3 def )
{
  Vector3 v;

  if (this->GetTupleString(key, 0, "") == "")
    return def;

  v.x = this->GetTupleLength(key, 0, 0.0);
  v.y = this->GetTupleLength(key, 1, 0.0);
  v.z = this->GetTupleLength(key, 2, 0.0);

  return v;
}

////////////////////////////////////////////////////////////////////////////
// Get a rotation
Quatern XMLConfigNode::GetRotation( std::string key, Quatern def )
{
  Quatern q;
  double px, py, pz;

  if (this->GetTupleString(key, 0, "") == "")
    return def;

  px = this->GetTupleAngle(key, 0, 0.0);
  py = this->GetTupleAngle(key, 1, 0.0);
  pz = this->GetTupleAngle(key, 2, 0.0);

  q.SetFromEuler(px, py, pz);

  return q;
}

////////////////////////////////////////////////////////////////////////////
// Get a tuple string value.
std::string XMLConfigNode::GetTupleString( std::string key, int index, 
                                           std::string def)
{
  xmlChar *value;
  std::string nvalue;
  int i, a, b, state, count;

  value = this->GetNodeValue( key );

  if (value == NULL)
    return def;

  state = 0;
  count = 0;
  a = b = 0;

  for (i = 0; i < (int) strlen((const char*) value); i++)
  {
    // Look for start of element
    if (state == 0)
    {
      if (!isspace( value[i] ))
      {
        a = i;
        state = 1;
      }
    }

    // Look for end of element
    else if (state == 1)
    {
      if (isspace( value[i] ))
      {
        state = 0;
        b = i - 1;
        count++;
        if (count > index)
          break;
      }
    }
  }

  if (state == 1)
  {
    b = i - 1;
    count++;
  }

  if (count == index + 1)
  {
    //nvalue = strndup( (const char*) value + a, b - a + 2 );
    const char *s = (const char*) value +a;
    size_t size = b-a+2;
    char *end = (char *)memchr(s,0,size);

    if (end)
      size = end -s + 1;

    char *r = (char *)malloc(size);

    if (size)
    {
      memcpy(r, s, size-1);
      r[size-1] = '\0';
    }

    nvalue = r;
  }
  
  xmlFree( value );

  return nvalue;
}

////////////////////////////////////////////////////////////////////////////
// Get an attribute tuple double value
int XMLConfigNode::GetTupleInt( std::string key, int index, int def )
{
  std::string svalue;

  svalue = this->GetTupleString( key, index, "" );
  if (svalue.empty())
    return def;

  return atoi(svalue.c_str());
}


////////////////////////////////////////////////////////////////////////////
// Get an attribute tuple double value
double XMLConfigNode::GetTupleDouble( std::string key, int index, double def )
{
  std::string svalue;

  svalue = this->GetTupleString( key, index, "" );
  if (svalue.empty())
    return def;

  return atof(svalue.c_str());
}


////////////////////////////////////////////////////////////////////////////
// Get an tuple length value (return value in meters)
double XMLConfigNode::GetTupleLength( std::string key, int index, double def )
{
  std::string svalue;

  svalue = this->GetTupleString( key, index, "" );
  if (svalue.empty())
    return def;

  // TODO: unit conversion here
  
  return atof(svalue.c_str());
}


////////////////////////////////////////////////////////////////////////////
// Get an tuple angle value (return value in radians)
double XMLConfigNode::GetTupleAngle( std::string key, int index, double def )
{
  std::string svalue;

  svalue = this->GetTupleString( key, index, "" );
  if (svalue.empty())
    return def;

  return atof(svalue.c_str()) * M_PI / 180;
}
