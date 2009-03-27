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

/* Desc: HTTP portal to libgazebo
 * Author: Brian Gerkey
 * Date: 9 March 2009
 * SVN: $Id: gazebo.h 7398 2009-03-09 07:21:49Z natepak $
 */

#include "webgazebo/WebGazebo.hh"

#include <stdlib.h>

#define USAGE "USAGE: webgazebo [-f <file.fed>] [-h <host>] [-p <port>] [-d <dtol>] [-a <atol>]\n"

#define DEFAULT_PORT 8000
#define DEFAULT_HOST "localhost"

int g_port = DEFAULT_PORT;
std::string g_host = DEFAULT_HOST;
std::string g_fedfile;
double g_dtol, g_atol;

bool ParseArgs(int argc, char** argv);

int
main(int argc, char** argv)
{
  if(!ParseArgs(argc, argv))
  {
    fputs(USAGE, stderr);
    exit(1);
  }

  WebGazebo wg(g_fedfile, g_host, g_port, g_dtol, g_atol);

  for(;;)
  {
    wg.Update();
    wg.Go(1.0);
  }

  return 0;
}

bool
ParseArgs(int argc, char** argv)
{
  char *flags = (char*)("f:p:h:d:a:");
  int ch;

  while ((ch = getopt(argc, argv, flags)) != -1)
  {
    switch(ch)
    {
      // federation file
      case 'f':
        if(!optarg)
          return false;
        g_fedfile = optarg;
        break;
      // port
      case 'p':
        if(!optarg)
          return false;
        g_port = atoi(optarg);
        break;
      // host
      case 'h':
        if(!optarg)
          return false;
        g_host = optarg;
        break;
      // distance tolerance
      case 'd':
        if(!optarg)
          return false;
        g_dtol = atof(optarg);
        break;
      // angular tolerance
      case 'a':
        if(!optarg)
          return false;
        g_atol = atof(optarg);
        break;
      default:
        return false;
    }
  }

  return true;
}
