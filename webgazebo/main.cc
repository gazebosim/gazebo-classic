/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/* Desc: HTTP portal to libgazebo
 * Author: Brian Gerkey, Richard Vaughan
 * Date: 9 March 2009
 * SVN: $Id: gazebo.h 7398 2009-03-09 07:21:49Z natepak $
 */

#include "WebGazebo.hh"

#include <stdlib.h>
#include <stdio.h>

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
  wg.Startup( true );

  for(;;)
  {
    //puts( "WGZ LOOP START" );
    //wg.Update();
    //wg.Go(1.0);
    //puts( "WGZ LOOP END" );
    wg.Wait();
    usleep( 1e3 );
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
