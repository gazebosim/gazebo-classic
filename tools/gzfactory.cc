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
#include <transport/transport.h>
#include <fstream>
#include <string>

using namespace gazebo;

void help()
{
  std::cout << "This tool spawns models into a running Gazebo simulation.\n\n"
            << "  gzfactory <world_name> <model_filename>\n"
            << "    world_name     : Name of the world in which to spawn "
            << "the model.\n"
            << "    modle_filename : Filename of the SDF model.\n\n";
}

int main(int argc, char **argv)
{
  if (argc < 3)
  {
    help();
    return -1;
  }

  std::ifstream ifs(argv[2]);
  if (!ifs)
  {
    printf("Error: Unable to open file[%s]\n", argv[2]);
    return -1;
  }
  std::string content((std::istreambuf_iterator<char>(ifs)),
                      (std::istreambuf_iterator<char>()));

  transport::init();

  transport::run();

  transport::NodePtr node(new transport::Node());
  node->Init(argv[1]);

  transport::PublisherPtr pub = node->Advertise<msgs::Factory>("~/factory");
  pub->WaitForConnection();

  msgs::Factory msg;
  msg.set_sdf(content);
  pub->Publish(msg, true);

  transport::fini();
}
