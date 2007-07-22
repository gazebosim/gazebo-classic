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
/* Desc: XML config file
 * Author: Andrew Howard and Nate Koenig
 * Date: 3 Apr 2007
 * SVN: $Id$
 */

#ifndef XMLCONFIG_HH
#define XMLCONFIG_HH

#include <libxml/parser.h>
#include <libxml/xpath.h>
#include <string>

#include "Vector3.hh"
#include "Vector2.hh"
#include "Quatern.hh"
#include "Time.hh"

namespace gazebo
{

// Forward declarations
class XMLConfigNode;

/// @addtogroup gazebocore
/// @{

/// \brief XML parser
class XMLConfig
{
  /// \brief Constructor
  public: XMLConfig();

  /// \brief Destructor
  public: ~XMLConfig();

  /// \brief Load config from an XML file
  public: int Load( std::string filename );

  /// \brief Load config from an XML string
  public: int LoadString( std::string str );

  /// \brief Save config back into file
  ///        Set filename to NULL to save back into the original file
  public: int Save( std::string filename );

  /// \brief Get the root node
  public: XMLConfigNode *GetRootNode() const;

  /// \brief Create wrappers
  private: XMLConfigNode *CreateNodes( XMLConfigNode *parent, 
                                       xmlNodePtr xmlNode );
  /// \brief File name
  public: std::string filename;
  
  /// \brief XML data
  private: xmlDocPtr xmlDoc;

  /// \brief The root of the tree
  private: XMLConfigNode *root;

  private: xmlXPathContextPtr xpathContex;
};


/// \brief Class encapsulating a single xml node
class XMLConfigNode
{
  /// \brief Constructor
  public: XMLConfigNode( XMLConfig *cfg, XMLConfigNode *parent, 
                        xmlNodePtr xmlNode, xmlDocPtr xmlDoc );

  /// \brief Destructors
  public: ~XMLConfigNode();

  /// \brief Get the node name
  public: std::string GetName();

  /// \brief Get the name space prefix
  public: std::string GetNSPrefix();

  /// \brief Get the next sibling of this node
  public: XMLConfigNode *GetNext();
  public: XMLConfigNode *GetNext(std::string name);
  public: XMLConfigNode *GetNextByNSPrefix(std::string name);

  /// \brief Get the first child of this node
  public: XMLConfigNode *GetChild();

  /// \brief Get a child based on a name. Returns null if not found
  public: XMLConfigNode *GetChild( std::string name );

  /// \brief Get the first child with the specified namespace prefix
  public: XMLConfigNode *GetChildByNSPrefix( std::string prefix );

  /// \brief Move child pointer back to beginning
  public: XMLConfigNode *Rewind();

  /// \brief Print (for debugging purposes)
  public: void Print();

  /// \brief Return the value of the current node
  public: std::string GetValue();

  /// \brief Get an attribute string value
  public: std::string GetString( std::string key, std::string def, 
                                 int require = 0 );

  /// \brief Get a attribute character value
  public: unsigned char GetChar( std::string key, char def, int require = 0 );

  /// \brief Get a file name.  Always returns an absolute path.  
  ///        If the filename is entered as a relative path, we prepend the 
  ///        world file path.
  public: std::string GetFilename( std::string key, std::string def, 
                                   int require = 0);

  /// \brief Get an integer
  public: int GetInt( std::string key, int def, int require = 0 );

  /// \brief Get a double
  public: double GetDouble( std::string key, double def, int require = 0 );

  /// \brief Get a float
  public: float GetFloat( std::string key, float def, int require = 0 );

  /// \brief Get a boolean
  public: bool GetBool( std::string key, bool def, int require = 0 );

  /// \brief Get an attribute length value (return value in meters)
  public: double GetLength( std::string key, double def, int require = 0 );

  /// \brief Get an attribute time value (return value in seconds)
  public: gazebo::Time GetTime( std::string key, double def, int require = 0 );

  /// \brief Get a position
  public: Vector3 GetVector3( std::string key, Vector3 def );

  /// \brief Get a two dimensional vector
  public: Vector2 GetVector2( std::string key, Vector2 def );

  /// \brief Get a rotation
  public: Quatern GetRotation( std::string key, Quatern def );

  /// \brief Get an attribute tuple value
  public: std::string GetTupleString( std::string key, int index, 
                                      std::string def );

  /// \brief Get an attribute tuple int value
  public: int GetTupleInt( std::string key, int index, int def );

  /// \brief Get an attribute tuple double value
  public: double GetTupleDouble( std::string key, int index, double def );

  /// \brief Get an attribute tuple length value (return value in meters)
  public: double GetTupleLength( std::string key, int index, double def );

  /// \brief Get an attribute tuple angle value (return value in radians)
  public: double GetTupleAngle( std::string key, int index, double def );

  /// \brief Get a node's value, which is either a attribute or child node value.
  protected: xmlChar* GetNodeValue( std::string key );

  /// \brief Our document
  private: XMLConfig *config;
  
  /// \brief Our parent
  private: XMLConfigNode *parent;
  
  /// \brief Our siblings
  private: XMLConfigNode *next, *prev;
  
  /// \brief Our children
  private: XMLConfigNode *childFirst, *childLast;

  /// \brief XML data
  private: xmlNodePtr xmlNode;

  /// \brief XML data
  private: xmlDocPtr xmlDoc;
};

/// @}
}

#endif

