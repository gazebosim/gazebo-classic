/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: XML file parser
 * Author: Andrew Howard and Nate Koenig
 * Date: 3 Apr 2007
 * SVN: $Id$
 */

#include <assert.h>
#include <float.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <sstream>

#include <libxml/xinclude.h>
#include <libxml/xpointer.h>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "Global.hh"
#include "XMLConfig.hh"

using namespace gazebo;

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
void XMLConfig::Load( const std::string &filename )
{
  this->filename = filename;

  std::ifstream fin;
  fin.open(this->filename.c_str(), std::ios::in);
  if( !fin.is_open() )
  {
    gzthrow( "The world file can not be opened, check path and permissions" );
  }
  fin.close();

  // Enable line numbering
  xmlLineNumbersDefault( 1 );

  // Load the file
  this->xmlDoc = xmlParseFile( this->filename.c_str() );
  this->FillDocumentNodes();
}

void XMLConfig::LoadString( const std::string &str )
{
  // Enable line numbering
  xmlLineNumbersDefault( 1 );

  // Load the file
  this->xmlDoc = xmlParseDoc( (xmlChar*)(str.c_str()) );
  this->FillDocumentNodes();
}


////////////////////////////////////////////////////////////////////////////
//  Save config back into file
void XMLConfig::Save(const std::string &filename )
{
  int result=0;
  if (filename == std::string())
    result=xmlSaveFileEnc(this->filename.c_str(), this->xmlDoc, "UTF-8");
  else
    result=xmlSaveFileEnc(filename.c_str(), this->xmlDoc, "UTF-8");
  if ( result != 0)
  {
     gzthrow( "Error saving the XML file back to the disk " );
  }
}


////////////////////////////////////////////////////////////////////////////
// Get the root node
XMLConfigNode *XMLConfig::GetRootNode() const
{
  return this->root;
}



////////////////////////////////////////////////////////////////////////////
// Fills the document with information 
void XMLConfig::FillDocumentNodes()
{
  if (xmlDoc == NULL)
  {
    gzthrow( "Unable to parse xml file: " << this->filename);
  }

  // Create xpath evaluation context
  this->xpathContex = xmlXPathNewContext(this->xmlDoc);
  if (this->xpathContex == NULL)
  {
    gzthrow("Unable to create new XPath context");
  }

  // Apply the XInclude process.
  if (xmlXIncludeProcess(this->xmlDoc) < 0)
  {
    //this will fail if the included file is not found, too strict?
    gzthrow("XInclude process failed\n");
  }

  // Create wrappers for all the nodes (recursive)
  this->root = this->CreateNodes( NULL, xmlDocGetRootElement(this->xmlDoc) );
  if (this->root == NULL)
  {
    gzthrow( "Empty document [" << this->filename << "]");
  }
}


////////////////////////////////////////////////////////////////////////////
// Create wrappers
XMLConfigNode *XMLConfig::CreateNodes( XMLConfigNode *parent,
                                       xmlNodePtr xmlNode )
{
  XMLConfigNode *self = NULL;

  // No need to create wrappers around text and blank nodes
  if ( !xmlNodeIsText( xmlNode ) && !xmlIsBlankNode( xmlNode ) &&
       strcmp((const char*)(xmlNode->name),"comment") )
  {

    // Create a new node
    self = new XMLConfigNode(this, parent, xmlNode,xmlDoc);

    // Create our children
    for ( xmlNode = xmlNode->xmlChildrenNode; xmlNode != NULL;
          xmlNode = xmlNode->next)
    {

      // If the node is xi:include, then only process the children of
      // the included XML files' root node
      if (!xmlNode->ns &&
          !strcmp((const char*)(xmlNode->name),"include"))
      {
        xmlNodePtr xmlNode2 = NULL;
        xmlChar *value = NULL;

        // Check to see if this model is meant to be embedded.
        // When embedded == true, then the root node of the included XML
        // file should be skipped.
        if (xmlHasProp( xmlNode, (xmlChar*)"embedded" ))
        {
          value = xmlGetProp( xmlNode, (xmlChar*)"embedded"  );
        }

        if (value && !strcmp((const char*)value,"true"))
        {
          // Get point to the first node of the included file
          xmlNode2 = xmlNode->xmlChildrenNode->next;

          // Skip over garbage
          while (xmlNodeIsText( xmlNode2 ) ||
                 !strcmp((const char*)xmlNode2->name,"comment") ||
                 !strcmp((const char*)xmlNode2->name,"include"))
            xmlNode2 = xmlNode2->next;

          // Get pointer to the first child of the included file
          xmlNode2 = xmlNode2->xmlChildrenNode;
          xmlFree(value);
        }
        else if (value)
        {
          // Get pointer to the first node in the included file.
          xmlNode2 = xmlNode->xmlChildrenNode->next;
          xmlFree(value);
        }

        // Process all children of the included file's root node
        for (; xmlNode2 != NULL; xmlNode2 = xmlNode2->next)
        {
          this->CreateNodes(self, xmlNode2);
        }

        // Now skip of the root node that was just processed
        xmlNode = xmlNode->next;
      }
      else
      {
        this->CreateNodes( self, xmlNode );
      }
    }
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

}


////////////////////////////////////////////////////////////////////////////
// Get the node name
std::string XMLConfigNode::GetName() const
{
  return (const char*)(this->xmlNode->name);
}

////////////////////////////////////////////////////////////////////////////
// Get the Name Space Prefix
std::string XMLConfigNode::GetNSPrefix() const
{
  if ( !this->xmlNode->ns )
    return "";
  else
    return (const char*)this->xmlNode->ns->prefix;
}

////////////////////////////////////////////////////////////////////////////
// Get the next sibling of this node
XMLConfigNode *XMLConfigNode::GetNext() const
{
  return this->next;
}

////////////////////////////////////////////////////////////////////////////
// Get the next sibling of this node
XMLConfigNode *XMLConfigNode::GetNext(const std::string &name, const std::string &prefix) const
{
  XMLConfigNode *tmp;

  for (tmp = this->next; tmp != NULL; tmp = tmp->GetNext() )
  {
    if (tmp->xmlNode->name && name == (const char*)tmp->xmlNode->name)
      if (prefix==std::string() || (prefix == (const char*)tmp->xmlNode->ns->prefix))
        break;
  }

  return tmp;
}

////////////////////////////////////////////////////////////////////////////
// Get the next sibling of this node according the namespace prefix
XMLConfigNode *XMLConfigNode::GetNextByNSPrefix(const std::string &prefix) const
{
  XMLConfigNode *tmp;

  for (tmp = this->next; tmp != NULL; tmp = tmp->GetNext() )
  {
    if (tmp->xmlNode->ns && prefix == (const char*)tmp->xmlNode->ns->prefix)
      break;
  }

  return tmp;

}

////////////////////////////////////////////////////////////////////////////
// Get the first child of this node
XMLConfigNode *XMLConfigNode::GetChild() const
{
  return this->childFirst;
}

////////////////////////////////////////////////////////////////////////////
// Get the first child with the appropriate NS prefix
XMLConfigNode *XMLConfigNode::GetChildByNSPrefix(const std::string &prefix ) const
{
  XMLConfigNode *tmp;

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
XMLConfigNode *XMLConfigNode::GetChild( const std::string &name, const std::string &prefix ) const
{
  XMLConfigNode *tmp;

  for (tmp = this->childFirst; tmp != NULL; tmp = tmp->GetNext() )
  {
    if (tmp->xmlNode->name && name == (const char*)tmp->xmlNode->name)
      if (prefix==std::string() || (prefix == (const char*)tmp->xmlNode->ns->prefix))
        break;
  }


//  for (tmp = this->childFirst; tmp != NULL &&
//      name != (const char*)tmp->xmlNode->name; tmp = tmp->GetNext() );

  return tmp;
}


////////////////////////////////////////////////////////////////////////////
// Print (for debugging purposes)
void XMLConfigNode::Print()
{
  XMLConfigNode *node;

  gzmsg(2) << "name = [" << (const char*) this->xmlNode->name << "]\n";

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
std::string XMLConfigNode::GetNodeValue( const std::string &key ) const
{
  std::string result;
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

  if (value)
  {
    result = (char*)value;
    boost::trim(result);

    xmlFree(value);
  }

  return result;
}


////////////////////////////////////////////////////////////////////////////
// Get the value of this node
std::string XMLConfigNode::GetValue() const
{
  return (const char*)xmlNodeListGetString(this->xmlDoc, this->xmlNode->xmlChildrenNode, 1);
}


////////////////////////////////////////////////////////////////////////////
// Get a string value.
std::string XMLConfigNode::GetString( const std::string &key, const std::string &def, int require) const 
{
  std::string value = this->GetNodeValue( key );

  if (value.empty() && require)
  {
    gzthrow( "unable to find required string attribute[" << key << "] in world file node[" << this->GetName() << "]");
  }
  else if ( value.empty() )
    return def;

  // TODO: cache the value somewhere (currently leaks)
  return value;
}

unsigned char XMLConfigNode::GetChar( const std::string &key, char def, int require ) const
{
  std::string value = this->GetNodeValue( key );

  if (value.empty() && require)
  {
    gzthrow("unable to find required char attribute[" << key << "] in world file node[" << this->GetName() << "]");
  }
  else if ( value.empty() )
    return def;

  return value[0];
}

///////////////////////////////////////////////////////////////////////////
// Get a file name.  Always returns an absolute path.  If the filename
// is entered as a relative path, we prepend the world file path.
// std::string XMLConfigNode::GetFilename( const std::string &key, const std::string &def, int require) const
// {
//   std::string filename = this->GetString( key, def, require );
//
//   if (filename.empty())
//     return "";
//
//   if (filename[0] == '/' || filename[0] == '~')
//     return filename;
//   else
//   {
//     std::string result;
//
//     if (this->config->filename[0] != '/' && this->config->filename[0] != '~')
//       result = "/";
//
//     unsigned int last = this->config->filename.rfind("/");
//     if (last==0 || last+1 != this->config->filename.size())
//       result += this->config->filename + "/" + filename;
//     else
//       result += this->config->filename.substr(0,last) + "/" + filename;
//
//     return result;
//   }
// }
///////////////////////////////////////////////////////////////////////////
// Get a file name.  Always returns an absolute path.  If the filename
// is entered as a relative path, we prepend the world file path.
//
// patch by stu to do relative path.  FIXME: what was the original implementation that's broken?
// sglaser: Was completely broken.  Now returns a path relative to the
// (original) working directory.
std::string XMLConfigNode::GetFilename( const std::string &key, const std::string &def, int require) const
{
  std::string filename = this->GetString( key, def, require );

  if (filename.empty() && require)
  {
    gzthrow("unable to find required filename attribute[" << key << "] in world file node["
            << this->GetName() << "]");
  }
  else if (filename.empty())
    return def;

  if (filename[0] == '/')
    return filename;
  else
  {
    int last_slash = this->config->filename.rfind("/");
    if (last_slash < 0)
      return filename;
    else
      return this->config->filename.substr(0,last_slash) + "/" + filename;
  }
}


////////////////////////////////////////////////////////////////////////////
// Get an integer
int XMLConfigNode::GetInt( const std::string &key, int def, int require ) const
{
  std::string value = this->GetNodeValue( key );

  if (value.empty() && require)
  {
    gzthrow ("unable to find required int attribute[" << key << "] in world file node[" << this->GetName() << "]");
  }
  else if ( value.empty() )
    return def;

  return boost::lexical_cast<int>(value);
}


////////////////////////////////////////////////////////////////////////////
// Get a double
double XMLConfigNode::GetDouble( const std::string &key, double def, int require ) const
{
  std::string value = this->GetNodeValue( key );

  if (value.empty() && require)
  {
    gzthrow( "unable to find required double attribute[" << key << "] in world file node[" << this->GetName() << "]");
  }
  else if ( value.empty() )
    return def;

  return boost::lexical_cast<double>(value);
}

////////////////////////////////////////////////////////////////////////////
// Get a float
float XMLConfigNode::GetFloat( const std::string &key, float def, int require ) const
{
  std::string value = this->GetNodeValue( key );

  if (value.empty() && require)
  {
    gzthrow( "unable to find required float attribute[" << key << "] in world file node[" << this->GetName() << "]");
  }
  else if ( value.empty() )
    return def;

  return boost::lexical_cast<float>(value);
}


////////////////////////////////////////////////////////////////////////////
// Get a boolean
bool XMLConfigNode::GetBool( const std::string &key, bool def, int require ) const
{
  bool result = false;
  std::string value = this->GetNodeValue( key );

  if (value.empty() && require)
  {
    gzthrow( "unable to find required bool attribute[" << key << "] in world file node[" << this->GetName() << "]");
  }
  else if ( value.empty() )
    return def;

  if (value == "true")
    result = true;
  else if (value == "false")
    result = false;
  else
    result = boost::lexical_cast<bool>(value);

  return result;
}


////////////////////////////////////////////////////////////////////////////
// Get a length
double XMLConfigNode::GetLength( const std::string &key, double def, int require ) const
{
  double length = this->GetDouble(key, def, require);

  // Do unit conversion here

  return length;
}


////////////////////////////////////////////////////////////////////////////
// Get a time
gazebo::Time XMLConfigNode::GetTime( const std::string &key, double def, int require ) const
{
  gazebo::Time time(this->GetDouble(key, def, require));
  return time;
}

////////////////////////////////////////////////////////////////////////////
// Get a position
Vector3 XMLConfigNode::GetVector3( const std::string &key, Vector3 def ) const
{
  Vector3 v;

  if (this->GetTupleString(key, 0, "") == "")
    return def;

  v.x = this->GetTupleDouble(key, 0, 0.0);
  v.y = this->GetTupleDouble(key, 1, 0.0);
  v.z = this->GetTupleDouble(key, 2, 0.0);

  return v;
}

////////////////////////////////////////////////////////////////////////////
// Get a two dimensional double vector
Vector2<double> XMLConfigNode::GetVector2d( const std::string &key, Vector2<double> def ) const
{
  Vector2<double> v;

  if (this->GetTupleString(key, 0, "") == "")
    return def;

  v.x = this->GetTupleDouble(key, 0, 0.0);
  v.y = this->GetTupleDouble(key, 1, 0.0);

  return v;
}

////////////////////////////////////////////////////////////////////////////
// Get a two dimensional int vector
Vector2<int> XMLConfigNode::GetVector2i( const std::string &key, Vector2<int> def ) const
{
  Vector2<int> v;

  if (this->GetTupleString(key, 0, "") == "")
    return def;

  v.x = this->GetTupleInt(key, 0, 0);
  v.y = this->GetTupleInt(key, 1, 0);

  return v;
}


////////////////////////////////////////////////////////////////////////////
// Get a rotation
Quatern XMLConfigNode::GetRotation( const std::string &key, Quatern def ) const
{
  Quatern q;
  Vector3 p;

  if (this->GetTupleString(key, 0, "") == "")
    return def;

  // Roll is around the x-axis
  p.x = this->GetTupleAngle(key, 0, 0.0);

  // Pitch is around the y-axis
  p.y = this->GetTupleAngle(key, 1, 0.0);

  // Yaw is around the z-axis
  p.z = this->GetTupleAngle(key, 2, 0.0);

  q.SetFromEuler(p);

  return q;
}

////////////////////////////////////////////////////////////////////////////
// Get a tuple string value.
std::string XMLConfigNode::GetTupleString( const std::string &key, int index,
                                           const std::string &def) const
{
  std::vector<std::string> split_vector;
  std::string value = this->GetNodeValue( key );
  boost::trim(value);
  boost::split(split_vector, value, boost::is_any_of(" "));

  if (index < (int)split_vector.size())
    return split_vector[index];
  else
    return def;
}

////////////////////////////////////////////////////////////////////////////
// Get an attribute tuple double value
int XMLConfigNode::GetTupleInt( const std::string &key, int index, int def ) const
{
  std::string svalue;

  svalue = this->GetTupleString( key, index, "" );
  if (svalue.empty())
    return def;

  return atoi(svalue.c_str());
}


////////////////////////////////////////////////////////////////////////////
// Get an attribute tuple double value
double XMLConfigNode::GetTupleDouble( const std::string &key, int index, double def ) const
{
  std::string svalue;

  svalue = this->GetTupleString( key, index, "" );
  if (svalue.empty())
    return def;

  return atof(svalue.c_str());
}


////////////////////////////////////////////////////////////////////////////
// Get an tuple length value (return value in meters)
double XMLConfigNode::GetTupleLength( const std::string &key, int index, double def ) const
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
double XMLConfigNode::GetTupleAngle( const std::string &key, int index, double def ) const
{
  std::string svalue;

  svalue = this->GetTupleString( key, index, "" );
  if (svalue.empty())
    return def;

  return DTOR(atof(svalue.c_str()));
}


////////////////////////////////////////////////////////////////////////////
// Set the value associated with a node.
/*void XMLConfigNode::SetValue(const std::string &key, const StringValue &data, int require, int type) 
{
  bool success;

  success = this->SetNodeValue(key, data.GetStr());

  if (!success && require)
  {
    this->NewNode(key.c_str(), data.GetCharStr(), type);
  }

}*/


////////////////////////////////////////////////////////////////////////////
bool XMLConfigNode::SetNodeValue(const std::string& key,const std::string& value)
{
  bool success=false;

  // First check if the key is an attribute
  if (xmlHasProp( this->xmlNode, (xmlChar*) key.c_str() ))
  {
    xmlSetProp( this->xmlNode, (xmlChar*) key.c_str(), (xmlChar*) value.c_str() );
    success=true;
  } // This very same node

  else if (key == this->GetName())
  {
    xmlNodeSetContent(this->xmlNode, (xmlChar*) value.c_str() );
    success=true;
  }// If not, then it should be a child node

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
        xmlNodeSetContent(currNode->xmlNode, (xmlChar*) value.c_str());
        success=true;
        break;
      }

      currNode = currNode->next;
    }
  }
  return success;
}


////////////////////////////////////////////////////////////////////////////
// Create a new node, child of this. May be a attribute or an element
void XMLConfigNode::NewNode(const char* key, const char* value, int type)
{
  if (type ==0) // A new child element
  {
    xmlNodePtr newNode;
    newNode = xmlNewNode(0, (xmlChar*) key); //I hope we don't need namespaces here
    if (!newNode)
    {
      gzthrow( "unable to create an element [" << key << "] in world file node[" << this->GetName() << "]");
    }
    xmlNodeSetContent(newNode, (xmlChar*) value);
    xmlAddChild(this->xmlNode, newNode);
  }

  else //a new atribute
  {
    xmlNewProp(this->xmlNode, (xmlChar*) key, (xmlChar*) value);
  }
}



