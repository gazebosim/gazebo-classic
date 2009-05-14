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
/* Desc: OpenAL API
 * Author: Nate Koenig
 * Date: 14 May 2009
 * SVN: $Id$
 */

#ifndef OPENALAPI_HH
#define OPENALAPI_HH

namespace gazebo
{
  class OpenALSource;
  class XMLConfigNode;

  class OpenALAPI
  {
    /// \brief Constructor
    public: OpenALAPI();

    /// \brief Destructor
    public: virtual ~OpenALAPI();

    /// \brief Load
    public: void Load(XMLConfigNode *node);

    /// \brief Initialize
    public: void Init();

    /// \brief Create an OpenALSource object from XML node
    /// \param node The XML config node containing the auiod information
    /// \return A pointer to an OpenALSource object
    //public: OpenALSource *CreateSource( XMLConfigNode *node );

    /// \brief Finalize
    public: void Fini();
  };


}

#endif
